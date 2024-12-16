#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import pyttsx3
import pyaudio
import re
from audio_compass.srv import TextToSpeech, TextToSpeechResponse


class SpeechGenerator:
    def __init__(self):
        """初始化ROS TTS节点"""
        rospy.init_node('speech_generator', anonymous=True)

        # TTS引擎状态
        self.engine = None
        self.voices = None
        self.is_engine_ready = False

        # 存储不同语言的声音设置
        self.voice_settings = {
            'en': {'rate': 175, 'voice_id': None},
            'zh': {'rate': 175, 'voice_id': None},
            'ja': {'rate': 175, 'voice_id': None}
        }

        # 初始化TTS引擎
        self._init_tts_engine()

        # 创建服务
        self.service = rospy.Service('text_to_speech', TextToSpeech, self.handle_tts_request)
        rospy.loginfo("Text To Speech(TTS) 服务已启动")

        # 创建定时器，定期检查音频设备
        self.check_timer = rospy.Timer(rospy.Duration(5.0), self._check_audio_timer_callback)

    def _init_tts_engine(self):
        """初始化TTS引擎"""
        try:
            if self._check_audio_output():
                self.engine = pyttsx3.init()
                self.voices = self.engine.getProperty('voices')
                self.engine.setProperty('volume', 1.0)
                self.is_engine_ready = True
                self._setup_voices()
                rospy.loginfo("TTS引擎初始化成功")
            else:
                self.is_engine_ready = False
                rospy.logwarn("未检测到音频设备，TTS功能暂时不可用")
        except Exception as e:
            self.is_engine_ready = False
            rospy.logwarn(f"TTS引擎初始化失败: {e}")

    def _check_audio_output(self):
        """检查是否有可用的音频输出设备"""
        try:
            p = pyaudio.PyAudio()
            output_device_count = p.get_host_api_info_by_index(0)['deviceCount']

            # 检查是否存在输出设备
            found_output = False
            for i in range(output_device_count):
                device_info = p.get_device_info_by_index(i)
                if device_info['maxOutputChannels'] > 0:
                    rospy.loginfo(f"找到音频输出设备: {device_info['name']}")
                    found_output = True
                    break

            p.terminate()
            return found_output
        except Exception as e:
            rospy.logwarn(f"检查音频输出设备时出错: {e}")
            return False

    def _check_audio_timer_callback(self, event):
        """定时检查音频设备的回调函数"""
        if not self.is_engine_ready:
            if self._check_audio_output():
                rospy.loginfo("检测到新的音频设备，重新初始化TTS引擎")
                self._init_tts_engine()

    def _setup_voices(self):
        """设置每种语言的声音"""
        if not self.is_engine_ready or not self.voices:
            return

        # 打印可用的声音列表（很长，不需要可以注释掉）
        # self._print_available_voices()

        for voice in self.voices:
            voice_id = voice.id.lower()
            if 'english' in voice_id or 'en' in voice_id:
                self.voice_settings['en']['voice_id'] = voice.id
            elif 'chinese' in voice_id or 'zh' in voice_id:
                # 强制选择普通话语音
                if 'mandarin' in voice.name.lower() or 'cmn' in voice.languages:
                    self.voice_settings['zh']['voice_id'] = voice.id
            elif 'japanese' in voice_id or 'ja' in voice_id or 'jp' in voice_id:
                self.voice_settings['ja']['voice_id'] = voice.id

    def _print_available_voices(self):
        """打印可用的声音列表"""
        rospy.loginfo("可用的声音:")
        for voice in self.voices:
            rospy.loginfo(f"ID: {voice.id}")
            rospy.loginfo(f"Name: {voice.name}")
            rospy.loginfo(f"Languages: {voice.languages}")
            rospy.loginfo("------------------------")

    def detect_language(self, text):
        """检测文本的主要语言"""
        japanese_pattern = r'[\u3040-\u309F\u30A0-\u30FF]'
        chinese_pattern = r'[\u4e00-\u9fff]'
        english_pattern = r'[a-zA-Z]'

        jp_chars = len(re.findall(japanese_pattern, text))
        cn_chars = len(re.findall(chinese_pattern, text))
        en_chars = len(re.findall(english_pattern, text))

        max_chars = max(jp_chars, cn_chars, en_chars)
        if max_chars == 0:
            return 'en'
        elif max_chars == jp_chars:
            return 'ja'
        elif max_chars == cn_chars:
            return 'zh'
        else:
            return 'en'

    def speak(self, text, language=None):
        """执行文本转语音"""
        if not self.is_engine_ready:
            rospy.logwarn("TTS引擎未就绪，跳过语音播放")
            return False

        try:
            if language is None:
                language = self.detect_language(text)

            if language not in self.voice_settings:
                rospy.logwarn(f"不支持的语言: {language}，将使用英语")
                language = 'en'

            settings = self.voice_settings[language]

            if settings['voice_id'] is None:
                rospy.logwarn(f"未找到{language}的声音，使用系统默认声音")
            else:
                self.engine.setProperty('voice', settings['voice_id'])

            self.engine.setProperty('rate', settings['rate'])

            rospy.loginfo(f"使用语言: {language}")
            self.engine.say(text)
            self.engine.runAndWait()
            return True

        except Exception as e:
            rospy.logerr(f"TTS转换错误: {str(e)}")
            return False

    def handle_tts_request(self, req):
        """处理 Text To Speech(TTS) 服务请求"""
        success = self.speak(req.text, req.language if req.language else None)
        message = "Success" if success else "Failed (No audio device or error occurred)"
        return TextToSpeechResponse(success=success, message=message)

    def cleanup(self):
        """清理资源"""
        if self.check_timer:
            self.check_timer.shutdown()
        if self.is_engine_ready and self.engine:
            try:
                self.engine.stop()
            except:
                pass

    def run(self):
        """运行节点"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            self.cleanup()


if __name__ == "__main__":
    try:
        speech_generator = SpeechGenerator()
        speech_generator.run()
    except rospy.ROSInterruptException:
        pass
