"""Speech Generator Package

This package provides text-to-speech functionality for ROS applications,
supporting multiple languages and TTS engines.
"""

from speech_generator.speech_generator import SpeechGenerator
from speech_generator.tts_engine import TTSEngine, EdgeTTSEngine, PyttsxEngine
from speech_generator.tts_cache import TTSCache
from speech_generator.language_detector import LanguageDetector

# Export public interfaces
__all__ = [
    'SpeechGenerator',      # Main speech generator class
    'TTSEngine',           # Abstract base class for TTS engines
    'EdgeTTSEngine',       # Edge TTS implementation
    'PyttsxEngine',        # pyttsx3 implementation
    'TTSCache',            # Cache management for TTS audio
    'LanguageDetector'     # Language detection utility
]