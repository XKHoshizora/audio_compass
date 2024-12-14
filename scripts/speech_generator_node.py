#!/usr/bin/env python3

import rospy
from speech_generator import SpeechGenerator

if __name__ == "__main__":
    try:
        speech_generator = SpeechGenerator()
        speech_generator.run()
    except rospy.ROSInterruptException:
        pass 
