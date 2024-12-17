#!/usr/bin/env python3

import rospy
from pathlib import Path
from speech_generator import SpeechGenerator

if __name__ == "__main__":
    try:
        config_path = rospy.get_param('~config_path', str(Path(__file__).parents[1] / "config" / "tts_config.yaml"))
        speech_generator = SpeechGenerator(config_path)
        speech_generator.run()
    except rospy.ROSInterruptException:
        pass
