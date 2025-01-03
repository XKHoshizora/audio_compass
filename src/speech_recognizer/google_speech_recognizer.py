#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Google Speech Recognizer"""
import rospy
import socket
import math
from threading import Thread
import re
import speech_recognition as sr
from audio_compass.msg import SpeechDirection
from speech_recognizer.base_recognizer import BaseRecognizer


class GoogleSpeechRecognizer(BaseRecognizer):
    def __init__(self, language='en-US', trigger_patterns=None):
        try:
            super().__init__()  # 调用基类初始化来检查设备

            # 添加自定义头部信息
            import platform
            self.recognizer = sr.Recognizer()
            self.recognizer.headers = {
                'User-Agent': f'Mozilla/5.0 ({platform.system()}; {platform.machine()}) speech_recognition/3.8.1',
                'Accept': 'text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8',
                'Accept-Language': 'en-US,en;q=0.5'
            }

            # 添加网络检查
            if not self.check_network_connection():
                self.log_err("网络连接失败，无法启动在线语音识别系统", show_terminal=True)
                return

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
                r'たっく',
                r'タックン',
                r'タッくん',
                r'たッくん',
                r'るみちゃん',
                r'ルミちゃん',
                # 其他模式...
            ]
            self.log_info(f"触发词模式已设置为: {self.trigger_patterns}")

            # 初始化识别器
            self.recognizer = sr.Recognizer()

            self.log_info("Google Speech Recognizer initialized")

        except Exception as e:
            self.log_err(f"Google Speech Recognizer 初始化失败: {str(e)}", show_terminal=True)

    def check_network_connection(self):
        """
        检查网络连接是否正常。
        尝试连接到公共 DNS 服务器（8.8.8.8），判断是否能够建立网络连接。
        """
        test_host = "8.8.8.8"  # Google Public DNS
        test_port = 53         # DNS 服务的标准端口
        timeout = 3            # 超时时间，单位为秒

        try:
            # 创建一个 socket 并尝试连接到指定的主机和端口
            with socket.create_connection((test_host, test_port), timeout):
                self.log_info("网络连接正常")
                return True
        except socket.timeout:
            self.log_err("网络连接超时，请检查网络", show_terminal=True)
        except socket.error as e:
            self.log_err(f"网络连接失败: {e}", show_terminal=True)
        return False

    def speech_recognize(self):
        """处理音频流并识别语音"""
        with sr.Microphone() as source:
            # 调整能源阈值以适应环境噪音
            self.recognizer.adjust_for_ambient_noise(source)

            self.log_info("请开始讲话...")

            # 循环处理音频流
            while not rospy.is_shutdown():
                try:
                    # 监听音频，设置超时时间和短语时间限制
                    audio = self.recognizer.listen(
                        source, timeout=5, phrase_time_limit=5)

                    try:
                        # 初始化 transcribed_text
                        transcribed_text = ""

                        # 只在调试模式下检查网络连接和API访问
                        if self.debug:
                            try:
                                import urllib.request
                                urllib.request.urlopen('http://www.google.com', timeout=1)
                                self.log_info("检查Google服务器连接...", show_terminal=False)
                            except Exception as e:
                                self.log_err(f"无法访问 Google 服务器: {str(e)}")
                                continue

                        # 尝试使用不同的语言代码格式
                        try_languages = [self.language, self.language.split('-')[0]]
                        for lang in try_languages:
                            try:
                                transcribed_text = self.recognizer.recognize_google(
                                    audio, language=lang)
                                if transcribed_text.strip():
                                    self.log_speech(f"识别结果: {transcribed_text}")
                                    # 如果成功，检查触发词
                                    if any(re.search(pattern, transcribed_text) for pattern in self.trigger_patterns):
                                        self.log_info("检测到触发词，发布方位信息...")
                                        self.publish_speech_direction(transcribed_text, -math.pi / 2)
                                break  # 如果成功则跳出循环
                            except sr.RequestError as e:
                                self.log_err(f"使用语言 {lang} 请求失败: {str(e)}")
                                continue

                    except sr.UnknownValueError:
                        self.log_warn("无语音输入或无法理解音频")
                    except sr.RequestError as e:
                        self.log_err(f"Google Speech API 请求失败: {str(e)}")
                        self.log_err(f"当前语言设置: {self.language}")

                except sr.WaitTimeoutError:
                    self.log_info("监听超时，未检测到语音")
                except Exception as e:
                    self.log_err(f"音频处理出错: {str(e)}", show_terminal=True)

                self.rate.sleep()

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
            self.log_err(f"语音识别系统运行出错: {str(e)}")
            self.cleanup()

    def cleanup(self):
        """清理资源"""
        self.log_info("语音识别系统已关闭")


if __name__ == "__main__":
    system = GoogleSpeechRecognizer()
    system.start()
