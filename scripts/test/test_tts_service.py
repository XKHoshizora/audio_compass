#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# test_tts_service.py

import rospy
from audio_compass.srv import TextToSpeech


def test_tts_service():
    rospy.init_node('test_tts_client')
    rospy.wait_for_service('/text_to_speech')

    try:
        # 1. 测试英文TTS
        tts = rospy.ServiceProxy('/text_to_speech', TextToSpeech)
        response = tts("Hello, this is a test.", "en")
        print(f"英文测试结果: {response.success}")

        # 2. 测试中文TTS
        response = tts("你好，这是一个测试。", "zh")
        print(f"中文测试结果: {response.success}")

        # 3. 测试日文TTS
        response = tts("こんにちは、テストです。", "ja")
        print(f"日文测试结果: {response.success}")

        # 4. 测试自动语言检测
        response = tts("这是中文测试", "")  # 空语言参数
        print(f"自动检测测试结果: {response.success}")

    except rospy.ServiceException as e:
        print(f"服务调用失败: {e}")


if __name__ == '__main__':
    test_tts_service()
