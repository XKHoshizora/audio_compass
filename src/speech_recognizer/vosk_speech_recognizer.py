#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import pyaudio
import math
from threading import Thread
import re
import json
from pathlib import Path
from vosk import Model, KaldiRecognizer
from audio_compass.msg import SpeechDirection
from speech_recognizer.base_recognizer import BaseRecognizer


class VoskSpeechRecognizer(BaseRecognizer):
    def __init__(self, language='en-US', trigger_patterns=None):
        try:
            super().__init__()  # 调用基类初始化来检查设备

            # 创建发布者
            self.pub = rospy.Publisher(
                'speech_direction', SpeechDirection, queue_size=1)

            # 设置循环频率
            self.rate = rospy.Rate(10)

            # 设置需要识别的语言
            self.language = language
            self.log_info(f"语言已设置为: {self.language}")

            # 扩展触发词模式，包括更多可能的变体
            self.trigger_patterns = trigger_patterns or [
                r'たっくん',
                r'タックン',
                r'タッくん',
                r'たッくん',
                r'るみちゃん',
                r'ルミちゃん',
                r'たっ くん',
                r'るみ ちゃん',
                r'ゆみ ちゃん',
                r'えみ ちゃん',
                r'レミ ちゃん',
                r'パッ クン',
                r'瑠美ちゃん',
                # 添加其他模式...
            ]

            # 初始化音频参数
            self.CHUNK = 1024
            self.FORMAT = pyaudio.paInt16
            self.CHANNELS = 1
            self.RATE = 16000
            self.p = pyaudio.PyAudio()

            # 当前脚本所在的目录
            current_dir = Path(__file__).parent
            # 模型路径
            model_path = Path(rospy.get_param('~model_path', str(current_dir.parents[2] / "models")))

            # 加载 Vosk 模型
            if self.language == 'en-US':
                self.model = Model(f"{model_path}/vosk-model-en-us-0.22")
            elif self.language == 'ja-JP':
                self.model = Model(f"{model_path}/vosk-model-ja-0.22")
            elif self.language == 'zh-CN':
                self.model = Model(f"{model_path}/vosk-model-cn-0.22")
            else:
                self.log_err(
                    f"不支持的语言: {self.language}，已默认设置为 'en-US'。其他语言请选择 'en-US', 'ja-JP' 或 'zh-CN' 。",
                    show_terminal=True
                )
                self.language = 'en-US'
                self.model = Model(f"{model_path}/vosk-model-en-us-0.22")

            self.log_info(f"Vosk 模型已加载: {self.model}")

            # 初始化语音识别器
            try:
                self.recognizer = KaldiRecognizer(self.model, self.RATE)
            except Exception as e:
                self.log_err(f"KaldiRecognizer 初始化失败: {str(e)}")
                raise

            self.log_info("Vosk Speech Recognizer initialized")

        except Exception as e:
            self.log_err(
                f"Vosk Speech Recognizer 初始化失败: {str(e)}", show_terminal=True)

    def start_audio_stream(self):
        """初始化并启动音频流"""
        # 启动主音频流
        self.stream = self.p.open(
            format=self.FORMAT,
            channels=self.CHANNELS,
            rate=self.RATE,
            input=True,
            frames_per_buffer=self.CHUNK
        )
        self.log_info(f"音频流已启动")

    def speech_recognize(self):
        """处理音频流并识别语音"""
        self.stream.start_stream()
        while not rospy.is_shutdown():
            try:
                data = self.stream.read(
                    self.CHUNK, exception_on_overflow=False)
                if len(data) == 0:
                    continue

                if self.recognizer.AcceptWaveform(data):
                    result = self.recognizer.Result()
                    res = json.loads(result)
                    transcribed_text = res.get("text", "")
                    if transcribed_text.strip():
                        self.log_speech(f"识别结果: {transcribed_text}")

                        # 检查是否包含触发词
                        if any(re.search(pattern, transcribed_text) for pattern in self.trigger_patterns):
                            self.log_warn("检测到触发词，发布方位信息...")
                            # 发布导航方向消息
                            self.publish_speech_direction(
                                transcribed_text, -math.pi / 2)  # 设置方向为正右
                else:
                    # 如果需要，可以处理部分结果
                    pass

            except Exception as e:
                self.log_err(f"音频处理出错: {str(e)}", show_terminal=True)
                self.rate.sleep()
        self.stream.stop_stream()

    def publish_speech_direction(self, recognized_text, direction):
        """发布导航方向消息"""
        msg = SpeechDirection()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.text = recognized_text
        msg.yaw = direction
        self.pub.publish(msg)

    def start(self):
        """启动语音识别系统"""
        try:
            self.log_info("启动语音识别系统...")
            self.start_audio_stream()

            # 启动音频处理线程
            audio_thread = Thread(target=self.speech_recognize)
            audio_thread.daemon = True  # 设置为守护线程，当主线程结束时结束该线程
            audio_thread.start()

            # 主线程保持运行，以监听 KeyboardInterrupt
            while not rospy.is_shutdown():
                self.rate.sleep()

        except KeyboardInterrupt:
            self.log_info("接收到停止信号，正在关闭语音识别系统...")
            self.cleanup()
        except Exception as e:
            self.log_err(f"语音识别系统运行出错: {str(e)}", show_terminal=True)
            self.cleanup()

    def cleanup(self):
        """清理资源"""
        if hasattr(self, 'stream') and self.stream.is_active():
            self.stream.stop_stream()
            self.stream.close()
        self.p.terminate()
        self.log_info("语音识别系统已关闭")


if __name__ == "__main__":
    system = VoskSpeechRecognizer()
    system.start()
