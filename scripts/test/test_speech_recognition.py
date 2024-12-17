#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# test_speech_recognition.py

import unittest
import rospy
from audio_compass.msg import SpeechDirection

class TestSpeechRecognition(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_speech_recognition')
        self.results = []
        self.sub = rospy.Subscriber(
            'speech_direction',
            SpeechDirection,
            self.callback
        )

    def callback(self, msg):
        self.results.append(msg)

    def test_recognition(self):
        # 等待消息
        rospy.sleep(5)
        self.assertTrue(len(self.results) > 0)

if __name__ == '__main__':
    unittest.main()