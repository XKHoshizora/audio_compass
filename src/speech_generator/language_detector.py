#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# language_detector.py

from typing import Dict, List
import re
from collections import defaultdict
import rospy
from langdetect import detect_langs, DetectorFactory
DetectorFactory.seed = 0  # 确保结果一致性


class LanguageDetector:
    """语言检测模块"""

    def __init__(self):
        # 语言特征字典
        self.features: Dict[str, List[str]] = {
            'zh': [
                r'[\u4e00-\u9fff]',  # 中文字符
                r'。|，|？|！|：|；'   # 中文标点
            ],
            'ja': [
                r'[\u3040-\u30ff]',  # 平假名和片假名
                r'[\u31f0-\u31ff]',  # 片假名语音扩展
                r'。|、|ー'           # 日文标点
            ],
            'en': [
                r'[a-zA-Z]+',        # 英文字母
                r'[.,!?:;]'          # 英文标点
            ]
        }

    def detect_language(self, text: str) -> str:
        """
        使用多种方法检测文本语言

        1. 首先使用特征匹配
        2. 然后使用langdetect库
        3. 最后使用置信度加权决策

        Args:
            text: 待检测的文本

        Returns:
            检测到的语言代码: 'zh', 'ja', 或 'en'
        """
        if not text:
            return 'en'

        # 1. 特征匹配分析
        feature_scores = self._analyze_features(text)

        # 2. 使用langdetect
        try:
            lang_probas = detect_langs(text)
            detected_lang = lang_probas[0].lang
            detected_prob = lang_probas[0].prob

            # 将langdetect的结果映射到我们的语言代码
            lang_mapping = {'zh-cn': 'zh', 'ja': 'ja', 'en': 'en'}
            detected_lang = lang_mapping.get(detected_lang, 'en')

            # 3. 综合两种方法的结果
            final_scores = {
                lang: score * 0.6 +
                (detected_prob if detected_lang == lang else 0) * 0.4
                for lang, score in feature_scores.items()
            }

            # 返回得分最高的语言
            return max(final_scores.items(), key=lambda x: x[1])[0]

        except Exception as e:
            rospy.logwarn(
                f"Language detection failed: {e}, using feature-based detection only")
            # 如果langdetect失败，只使用特征匹配结果
            return max(feature_scores.items(), key=lambda x: x[1])[0]

    def _analyze_features(self, text: str) -> Dict[str, float]:
        """分析文本特征"""
        scores = defaultdict(float)
        total_chars = len(text)

        for lang, patterns in self.features.items():
            lang_chars = 0
            for pattern in patterns:
                matches = re.findall(pattern, text)
                lang_chars += sum(len(match) for match in matches)

            if total_chars > 0:
                scores[lang] = lang_chars / total_chars

        return scores
