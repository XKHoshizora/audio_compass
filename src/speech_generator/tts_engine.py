#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# tts_engine.py

from abc import ABC, abstractmethod
import asyncio
import subprocess
from pathlib import Path
from typing import Dict, Optional
import edge_tts
# from TTS.api import TTS  # Coqui TTS
import pyttsx3
import rospy
from speech_generator.tts_cache import TTSCache


class TTSEngine(ABC):
    """TTS引擎的抽象基类"""
    @abstractmethod
    def initialize(self) -> bool:
        """初始化TTS引擎"""
        pass

    @abstractmethod
    def speak(self, text: str, voice_id: str, rate: str, volume: str) -> bool:
        """执行文本转语音"""
        pass

    @abstractmethod
    def stop(self) -> None:
        """停止当前播放"""
        pass


# class CoquiTTSEngine(TTSEngine):
#     """Coqui TTS引擎实现"""

#     def initialize(self) -> bool:
#         try:
#             self.tts = TTS(model_name="tts_models/en/ljspeech/tacotron2-DDC", progress_bar=False)
#             return True
#         except Exception as e:
#             rospy.logerr(f"Coqui TTS 初始化失败: {e}")
#             return False

#     def speak(self, text: str, voice_id: str, rate: str, volume: str) -> bool:
#         try:
#             output_path = "/tmp/coqui_output.wav"
#             self.tts.tts_to_file(text=text, file_path=output_path)
#             self._play_audio(output_path)
#             return True
#         except Exception as e:
#             rospy.logerr(f"Coqui TTS 播放失败: {e}")
#             return False


# class MozillaTTSEngine(TTSEngine):
#     """Mozilla TTS引擎实现"""

#     def initialize(self) -> bool:
#         self.model_path = "/path/to/model.pth"  # 替换为实际路径
#         self.config_path = "/path/to/config.json"
#         self.vocoder_path = "/path/to/vocoder.pth"
#         return True

#     def speak(self, text: str, voice_id: str, rate: str, volume: str) -> bool:
#         try:
#             output_path = "/tmp/mozilla_output.wav"
#             subprocess.run([
#                 "python3", "TTS/bin/synthesize.py",
#                 "--text", text,
#                 "--config_path", self.config_path,
#                 "--out_path", output_path
#             ])
#             self._play_audio(output_path)
#             return True
#         except Exception as e:
#             rospy.logerr(f"Mozilla TTS 播放失败: {e}")
#             return False


class EdgeTTSEngine(TTSEngine):
    """Edge TTS引擎实现"""

    def __init__(self, cache: TTSCache):
        self.cache = cache

    def initialize(self) -> bool:
        try:
            return True
        except Exception as e:
            rospy.logerr(f"Edge TTS初始化失败: {str(e)}")
            return False

    def speak(self, text: str, voice_id: str, rate: str, volume: str) -> bool:
        try:
            # 检查缓存
            cached_path = self.cache.get_cached_audio(
                text, voice_id, rate, volume)
            if cached_path:
                rospy.loginfo("Using cached audio")
                self._play_audio(cached_path)
                return True

            # 使用异步方式执行Edge TTS
            return asyncio.run(self._async_speak(text, voice_id, rate, volume))
        except Exception as e:
            rospy.logerr(f"Edge TTS播放失败: {str(e)}")
            return False

    async def _async_speak(self, text: str, voice_id: str, rate: str, volume: str) -> bool:
        try:
            # 规范化音量格式
            if not volume.startswith('+') and not volume.startswith('-'):
                volume = f"+{volume}" if volume.rstrip('%').isdigit() and int(volume.rstrip('%')) >= 100 else f"{int(volume.rstrip('%'))-100}%"

            # 创建临时文件路径
            output_file = Path("/tmp/output.mp3")

            # 创建Edge TTS通信对象
            communicate = edge_tts.Communicate(
                text=text,
                voice=voice_id,
                rate=rate,
                volume=volume
            )

            # 保存音频文件
            await communicate.save(str(output_file))

            # 缓存音频文件
            cached_path = self.cache.cache_audio(
                text, voice_id, rate, volume, str(output_file))

            # 播放音频
            self._play_audio(cached_path)

            # 清理临时文件
            if output_file.exists() and str(output_file) != cached_path:
                output_file.unlink()
            return True
        except Exception as e:
            rospy.logerr(f"Edge TTS异步播放失败: {str(e)}")
            return False

    def _play_audio(self, audio_path: str) -> None:
        """播放音频文件"""
        subprocess.run(
            ["mpg123", audio_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

    def stop(self) -> None:
        rospy.logwarn("Edge TTS不支持停止功能")


class PyttsxEngine(TTSEngine):
    """pyttsx3引擎实现"""

    def __init__(self):
        self.engine = None

    def initialize(self) -> bool:
        try:
            self.engine = pyttsx3.init()
            return True
        except Exception as e:
            rospy.logerr(f"pyttsx3初始化失败: {str(e)}")
            return False

    def speak(self, text: str, voice_id: str, rate: str, volume: str) -> bool:
        try:
            if not self.engine:
                return False

            # 设置语音参数
            self.engine.setProperty('voice', voice_id)
            rate_value = int(rate.strip('%+'))
            self.engine.setProperty('rate', rate_value)

            # 设置音量 (0.0-1.0)
            volume_value = float(volume.strip('%')) / 100
            self.engine.setProperty('volume', volume_value)

            self.engine.say(text)
            self.engine.runAndWait()
            return True
        except Exception as e:
            rospy.logerr(f"pyttsx3播放失败: {str(e)}")
            return False

    def stop(self) -> None:
        if self.engine:
            self.engine.stop()
