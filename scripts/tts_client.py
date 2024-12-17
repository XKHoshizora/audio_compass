#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import rospy
from audio_compass.srv import TextToSpeech
from typing import Optional, Callable


class TTSClient:
    """
    文本转语音(TTS)客户端API封装

    提供同步和异步两种方式调用TTS服务，支持多语言和回调函数。

    示例:
        # 同步调用
        client = TTSClient()
        success = client.speak("你好世界")

        # 异步调用
        def callback(success, message):
            print(f"TTS完成: {'成功' if success else '失败'}, {message}")

        client = TTSClient()
        client.speak_async("Hello world", callback=callback)

        # 使用上下文管理器
        with TTSClient() as tts:
            tts.speak("自动管理连接和断开")
    """

    def __init__(self, service_name: str = '/text_to_speech', timeout: int = 10):
        """
        初始化TTS客户端

        Args:
            service_name: TTS服务名称
            timeout: 服务连接超时时间(秒)
        """
        self._service_name = service_name
        self._timeout = timeout
        self._client = None
        self._lock = threading.Lock()
        self._speaking_threads = set()

    def __enter__(self):
        """上下文管理器入口"""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器退出"""
        self.disconnect()

    def connect(self) -> bool:
        """
        连接到TTS服务

        Returns:
            bool: 连接是否成功
        """
        try:
            rospy.wait_for_service(self._service_name, timeout=self._timeout)
            self._client = rospy.ServiceProxy(self._service_name, TextToSpeech)
            return True
        except rospy.ROSException as e:
            rospy.logerr(f"无法连接到TTS服务: {e}")
            return False

    def disconnect(self):
        """断开与TTS服务的连接"""
        if self._client:
            self._client.close()
            self._client = None

        # 等待所有正在进行的语音合成完成
        threads = list(self._speaking_threads)
        for thread in threads:
            if thread.is_alive():
                thread.join(timeout=1)

    def _ensure_connected(self):
        """确保已连接到服务"""
        if not self._client:
            if not self.connect():
                raise RuntimeError("未连接到TTS服务")

    def speak(self, text: str, language: str = "") -> bool:
        """
        同步方式执行语音合成

        Args:
            text: 要转换的文本
            language: 语言代码 (可选, 如 'en', 'zh', 'ja')

        Returns:
            bool: 语音合成是否成功

        Raises:
            RuntimeError: 未连接到服务时抛出
            ROSException: ROS服务调用失败时抛出
        """
        self._ensure_connected()

        try:
            with self._lock:  # 防止并发调用服务
                response = self._client(text=text, language=language)
                return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"TTS服务调用失败: {e}")
            return False

    def speak_async(self,
                   text: str,
                   language: str = "",
                   callback: Optional[Callable[[bool, str], None]] = None) -> threading.Thread:
        """
        异步方式执行语音合成

        Args:
            text: 要转换的文本
            language: 语言代码 (可选)
            callback: 完成回调函数，接收success和message两个参数

        Returns:
            Thread: 执行语音合成的线程对象
        """
        def _speak_thread():
            try:
                success = self.speak(text, language)
                message = "Success" if success else "Failed"

                if callback:
                    callback(success, message)

            finally:
                self._speaking_threads.remove(thread)

        thread = threading.Thread(target=_speak_thread)
        self._speaking_threads.add(thread)
        thread.daemon = True  # 设置为守护线程
        thread.start()

        return thread

    def wait_for_all(self, timeout: Optional[float] = None):
        """
        等待所有异步语音合成完成

        Args:
            timeout: 超时时间(秒)，None表示无限等待
        """
        threads = list(self._speaking_threads)
        for thread in threads:
            if thread.is_alive():
                thread.join(timeout=timeout)

    @property
    def is_busy(self) -> bool:
        """当前是否有正在进行的语音合成"""
        return any(thread.is_alive() for thread in self._speaking_threads)


# 使用示例
if __name__ == "__main__":
    rospy.init_node('tts_client_test')

    def speak_callback(success, message):
        rospy.loginfo(f"语音完成: {'成功' if success else '失败'}, {message}")

    # 创建客户端
    client = TTSClient()

    # 测试同步调用
    success = client.speak("这是一个同步测试")
    rospy.loginfo(f"同步调用结果: {'成功' if success else '失败'}")

    # 测试异步调用
    client.speak_async("这是第一个异步测试", callback=speak_callback)
    client.speak_async("这是第二个异步测试", callback=speak_callback)

    # 等待所有异步调用完成
    client.wait_for_all()

    # 使用上下文管理器
    with TTSClient() as tts:
        tts.speak("使用上下文管理器测试")

    rospy.loginfo("测试完成")