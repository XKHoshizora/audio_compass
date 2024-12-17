#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# tts_engine.py

from abc import ABC, abstractmethod
import asyncio
import edge_tts
from pathlib import Path
import pyttsx3
import subprocess
import rospy
from typing import Dict, Optional


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
            output_file = Path("/tmp/output.mp3")
            communicate = edge_tts.Communicate(
                text=text,
                voice=voice_id,
                rate=rate,
                volume=volume
            )
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
