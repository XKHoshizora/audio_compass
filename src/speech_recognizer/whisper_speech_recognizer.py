#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import whisper
import pyaudio
import numpy as np
import rospy
import math
from threading import Thread
import re
import torch
from audio_compass.msg import SpeechDirection
from speech_recognizer.base_recognizer import BaseRecognizer


class WhisperSpeechRecognizer(BaseRecognizer):
    def __init__(self, language='en-US', trigger_patterns=None):
        try:
            super().__init__()  # 调用基类初始化来检查设备

            # 创建发布者
            self.pub = rospy.Publisher(
                'speech_direction', SpeechDirection, queue_size=1)

            # 设置循环频率
            self.rate = rospy.Rate(10)

            # 设置语言
            self.language = language.split('-')[0].lower()  # 转换为 Whisper 支持的格式（如 'en'）
            self.log_info(f"语言已设置为: {self.language}")

            # 设置设备
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
            self.log_info(f"使用设备: {self.device}")

            # 加载模型，可选：tiny, base, small, medium, large
            self.model = whisper.load_model("tiny", device=self.device)

            # 使用传入的触发词模式或默认值
            self.trigger_patterns = trigger_patterns or [
                r'たっくん',
                r'タックン',
                r'タッくん',
                r'たッくん',
                r'たつくん',
                r'タツクン',
                r'だっくん',
                r'ダックン',
                r'たくん',
                r'ダッグン',
                r'っくん',
                r'タッカン',
                r'タッフン',
                r'タック',    # 部分匹配
                r'たっく',    # 部分匹配
                r'ダック',    # 相似音
                r'タッグ',    # 相似音
                r'スタック',  # 包含关键音
                r'タク'      # 基本音
            ]

            # 初始化音频参数
            self.CHUNK = 1024 * 4
            self.FORMAT = pyaudio.paFloat32
            self.CHANNELS = 1
            self.RATE = 16000
            self.p = pyaudio.PyAudio()

            # 系统状态
            self.is_running = True

            self.log_info("Whisper Speech Recognizer initialized")

        except Exception as e:
            self.log_err(f"Whisper Speech Recognizer 初始化失败: {str(e)}")

    def start_audio_stream(self):
        """初始化并启动音频流"""
        self.stream = self.p.open(
            format=self.FORMAT,
            channels=self.CHANNELS,
            rate=self.RATE,
            input=True,
            frames_per_buffer=self.CHUNK
        )
        self.log_info("音频流已启动")

    def publish_speech_direction(self, recognized_text, direction):
        """发布导航方向消息"""
        msg = SpeechDirection()
        msg.header.stamp = rospy.Time.now()
        msg.text = recognized_text
        msg.yaw = direction
        self.pub.publish(msg)

    def speech_recognize(self):
        """处理音频流并识别语音"""
        self.stream.start_stream()
        while not rospy.is_shutdown() and self.is_running:
            try:
                # 采集3秒的音频数据
                frames = []
                for _ in range(0, int(self.RATE / self.CHUNK * 3)):
                    data = self.stream.read(self.CHUNK, exception_on_overflow=False)
                    frames.append(np.frombuffer(data, dtype=np.float32))

                # 合并音频帧
                audio_data = np.concatenate(frames)

                # 归一化到 [-1.0, 1.0] 范围
                audio_data = audio_data / np.max(np.abs(audio_data))

                # 使用Whisper进行识别
                result = self.model.transcribe(
                    audio_data,
                    language=self.language,
                )
                transcribed_text = result["text"]

                # 如果识别到文本，显示出来
                if transcribed_text.strip():
                    self.log_speech(f"识别结果: {transcribed_text}")

                    # 检查是否包含触发词
                    if any(re.search(pattern, transcribed_text) for pattern in self.trigger_patterns):
                        self.log_warn("检测到触发词，发布方位信息...")
                        # 发布导航方向消息
                        self.publish_speech_direction(transcribed_text, -math.pi / 2)  # 设置方向为正右

            except Exception as e:
                self.log_err(f"音频处理出错: {str(e)}")
                self.rate.sleep()

        self.stream.stop_stream()

    def start(self):
        """启动语音识别系统"""
        try:
            self.log_info("启动语音识别系统...")
            self.start_audio_stream()

            # 启动音频处理线程
            audio_thread = Thread(target=self.speech_recognize)
            audio_thread.daemon = True
            audio_thread.start()

            # 主线程保持运行，以监听 KeyboardInterrupt
            while not rospy.is_shutdown():
                self.rate.sleep()

        except KeyboardInterrupt:
            self.log_info("接收到停止信号，正在关闭语音识别系统...")
            self.cleanup()
        except Exception as e:
            self.log_err(f"语音识别系统运行出错: {str(e)}")
            self.cleanup()

    def cleanup(self):
        """清理资源"""
        self.is_running = False
        if hasattr(self, 'stream') and self.stream.is_active():
            self.stream.stop_stream()
            self.stream.close()
        self.p.terminate()
        self.log_info("语音识别系统已关闭")


if __name__ == "__main__":
    system = WhisperSpeechRecognizer()
    system.start()