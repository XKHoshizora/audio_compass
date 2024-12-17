#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# speech_generator.py

import yaml
import platform
from pathlib import Path
from typing import Dict, Any, Optional
import rospy
from std_srvs.srv import Empty, EmptyResponse
from audio_compass.srv import TextToSpeech, TextToSpeechResponse
from dynamic_reconfigure.server import Server
from audio_compass.cfg import TTSConfig
from dynamic_reconfigure.encoding import extract_params
from speech_generator.tts_engine import TTSEngine, EdgeTTSEngine, PyttsxEngine
from speech_generator.tts_cache import TTSCache
from speech_generator.language_detector import LanguageDetector


class SpeechGenerator:
    """语音生成器主类"""

    def __init__(self, config_path: str):
        # 首先初始化ROS节点
        rospy.init_node("speech_generator", anonymous=True)

        # 然后加载配置和初始化其他组件
        self.config = self._load_config(config_path)
        self.cache = TTSCache()
        self.engine = self._create_engine()
        self.language_detector = LanguageDetector()

        # 设置动态重配置服务器
        self.dyn_server = Server(TTSConfig, self.dynamic_reconfigure_callback)

        # 初始化服务
        rospy.Service("/text_to_speech", TextToSpeech, self.handle_tts_request)
        rospy.Service("/stop_tts", Empty, self.handle_tts_stop)

        # 定期清理缓存的计时器
        rospy.Timer(rospy.Duration(3600), self._clean_cache_callback)  # 每小时清理一次

        rospy.loginfo("Speech Generator Node 初始化完成.")

    def _load_config(self, config_path: str) -> Dict[str, Any]:
        """加载配置文件"""
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                return yaml.safe_load(f)
        except Exception as e:
            rospy.logerr(f"配置文件加载失败: {str(e)}")
            return {}

    def _create_engine(self) -> TTSEngine:
        """创建TTS引擎实例"""
        if platform.system() == "Linux":
            engine = EdgeTTSEngine(self.cache)
        else:
            engine = PyttsxEngine()

        if not engine.initialize():
            rospy.logerr("TTS引擎初始化失败")
        return engine

    def dynamic_reconfigure_callback(self, config, _):
        """处理动态重配置请求"""
        for lang in ['en', 'zh', 'ja']:
            if lang in self.config['tts_settings']:
                # 正确格式化rate
                rate_value = f"+{config[f'{lang}_rate']}%" if config[f'{lang}_rate'] >= 0 else f"{config[f'{lang}_rate']}%"
                # 正确格式化volume (Edge TTS需要+/-格式)
                volume_value = f"+{config[f'{lang}_volume']}%" if config[f'{lang}_volume'] >= 100 else f"{config[f'{lang}_volume']-100}%"

                self.config['tts_settings'][lang].update({
                    'rate': rate_value,
                    'volume': volume_value
                })
        return config

    def _clean_cache_callback(self, _):
        """定期清理缓存的回调函数"""
        self.cache.clean_old_cache()

    def speak(self, text: str, language: Optional[str] = None) -> bool:
        """执行文本转语音"""
        if not text:
            rospy.logwarn("收到空文本")
            return False

        # 检测语言
        detected_language = language or self.language_detector.detect_language(text)

        # 获取语音设置
        settings = self.config['tts_settings'].get(detected_language,  self.config['tts_settings']['en'])

        return self.engine.speak(
            text,
            settings['voice_id'],
            settings['rate'],
            settings.get('volume', '100%')  # 默认音量100%
        )

    def handle_tts_request(self, req: TextToSpeech) -> TextToSpeechResponse:
        """处理TTS服务请求"""
        try:
            success = self.speak(req.text, req.language if req.language else None)
            message = "Success" if success else "Failed"
            return TextToSpeechResponse(success=success, message=message)
        except Exception as e:
            rospy.logerr(f"TTS request failed: {str(e)}")
            return TextToSpeechResponse(success=False, message=str(e))

    def handle_tts_stop(self, _: Empty) -> EmptyResponse:
        """处理停止TTS请求"""
        self.engine.stop()
        return EmptyResponse()

    def run(self) -> None:
        """运行ROS节点"""
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("Speech Generator Node 被中断.")
        finally:
            self.cleanup()

    def cleanup(self):
        """清理资源"""
        if hasattr(self, 'engine'):
            self.engine.stop()
        rospy.loginfo("Speech Generator Node 已关闭.")


if __name__ == "__main__":
    config_path = rospy.get_param('~config_path', str(
        Path(__file__).parents[2] / "config" / "tts_config.yaml"))
    speech_generator = SpeechGenerator(config_path)
    speech_generator.run()
