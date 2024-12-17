#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from pathlib import Path
from speech_generator import SpeechGenerator

if __name__ == "__main__":
    try:
        # 获取配置文件路径
        config_path = rospy.get_param('~config_path', str(Path(__file__).parents[1] / "config" / "tts_config.yaml"))

        # 创建并运行语音生成器
        speech_generator = SpeechGenerator(config_path)
        speech_generator.run()
    except Exception as e:
        rospy.logerr(f"Speech Generator 启动失败: {str(e)}")
    except rospy.ROSInterruptException:
        pass
