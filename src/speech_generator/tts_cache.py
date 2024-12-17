#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# tts_cache.py

from pathlib import Path
import hashlib
import json
from threading import Lock
from typing import Dict, Optional
import rospy


class TTSCache:
    """TTS音频缓存管理"""

    def __init__(self, cache_dir: str = "/tmp/tts_cache"):
        self._cache_lock = Lock()
        self.cache_dir = Path(cache_dir)
        self.cache_info: Dict[str, Dict] = {}
        self.cache_file = self.cache_dir / "cache_info.json"
        self._init_cache()

    def _init_cache(self) -> None:
        """初始化缓存目录和加载缓存信息"""
        try:
            self.cache_dir.mkdir(parents=True, exist_ok=True)
            if self.cache_file.exists():
                with self.cache_file.open('r') as f:
                    self.cache_info = json.load(f)
        except Exception as e:
            rospy.logwarn(f"Cache initialization failed: {e}")
            self.cache_info = {}

    def _save_cache_info(self) -> None:
        """保存缓存信息到文件"""
        try:
            with self.cache_file.open('w') as f:
                json.dump(self.cache_info, f)
        except Exception as e:
            rospy.logwarn(f"Failed to save cache info: {e}")

    def _generate_cache_key(self, text: str, voice_id: str, rate: str, volume: str) -> str:
        """生成缓存键"""
        content = f"{text}_{voice_id}_{rate}_{volume}"
        return hashlib.md5(content.encode()).hexdigest()

    def get_cached_audio(self, text: str, voice_id: str, rate: str, volume: str) -> Optional[Path]:
        """获取缓存的音频文件路径"""
        cache_key = self._generate_cache_key(text, voice_id, rate, volume)
        file_path = self.cache_dir / f"{cache_key}.mp3"
        if file_path.exists():
            return file_path
        return None

    def cache_audio(self, text: str, voice_id: str, rate: str, volume: str, audio_path: str) -> Path:
        """缓存音频文件"""
        with self._cache_lock:
            cache_key = self._generate_cache_key(text, voice_id, rate, volume)
            cache_path = self.cache_dir / f"{cache_key}.mp3"

            try:
                with Path(audio_path).open('rb') as src, cache_path.open('wb') as dst:
                    dst.write(src.read())

                self.cache_info[cache_key] = {
                    'text': text,
                    'voice_id': voice_id,
                    'rate': rate,
                    'volume': volume,
                    'created_at': rospy.get_time()
                }
                self._save_cache_info()
            except Exception as e:
                rospy.logwarn(f"Failed to cache audio: {e}")
            return cache_path

    def clean_old_cache(self, max_age: float = 7 * 24 * 3600) -> None:
        """基于使用频率和时间清理缓存"""
        current_time = rospy.get_time()
        sorted_cache = sorted(self.cache_info.items(), key=lambda x: x[1].get('access_count', 0), reverse=True)

        for cache_key, info in sorted_cache:
            file_path = self.cache_dir / f"{cache_key}.mp3"
            file_age = current_time - info.get('created_at', 0)
            if file_age > max_age:
                try:
                    if file_path.exists():
                        file_path.unlink()
                    self.cache_info.pop(cache_key, None)
                    rospy.loginfo(f"清理过期缓存: {file_path}")
                except Exception as e:
                    rospy.logwarn(f"删除缓存文件失败: {e}")
        self._save_cache_info()
