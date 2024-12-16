import rospy
import os
import platform
import pyttsx3
import asyncio
import edge_tts
from std_srvs.srv import Empty, EmptyResponse
from audio_compass.srv import TextToSpeech, TextToSpeechResponse

class SpeechGenerator:
    def __init__(self):
        self.engine = None
        self.voices = []
        self.voice_settings = {
            'en': {'voice_id': 'en-US-JennyNeural', 'rate': 0},  # 正常语速
            'zh': {'voice_id': 'zh-CN-XiaoxiaoNeural', 'rate': 0},  # 正常语速
            'ja': {'voice_id': 'ja-JP-NanamiNeural', 'rate': 0}  # 正常语速
        }
        self.is_engine_ready = False
        self.use_edge_tts = platform.system() == "Linux"  # Ubuntu切换为Edge-TTS
        self._init_tts_engine()

    def _init_tts_engine(self):
        """初始化TTS引擎"""
        try:
            if self.use_edge_tts:
                # 初始化Edge-TTS
                self.engine = "edge-tts"
                self.is_engine_ready = True
                self._setup_voices_edge_tts()
                rospy.loginfo("Edge-TTS引擎初始化成功")
            else:
                # 初始化pyttsx3引擎
                self.engine = pyttsx3.init()
                self.voices = self.engine.getProperty('voices')
                self.is_engine_ready = True
                self._setup_voices_pyttsx3()
                rospy.loginfo("pyttsx3引擎初始化成功")
        except Exception as e:
            self.is_engine_ready = False
            rospy.logwarn(f"TTS引擎初始化失败: {e}")

    def _setup_voices_pyttsx3(self):
        """设置pyttsx3语音"""
        for voice in self.voices:
            voice_id = voice.id.lower()
            if 'english' in voice_id or 'en' in voice_id:
                self.voice_settings['en']['voice_id'] = voice.id
            elif 'chinese' in voice_id or 'zh' in voice_id:
                self.voice_settings['zh']['voice_id'] = voice.id
            elif 'japanese' in voice_id or 'ja' in voice_id or 'jp' in voice_id:
                self.voice_settings['ja']['voice_id'] = voice.id

    def _setup_voices_edge_tts(self):
        """设置Edge-TTS语音"""
        self.voice_settings['en']['voice_id'] = 'en-US-JennyNeural'
        self.voice_settings['zh']['voice_id'] = 'zh-CN-XiaoxiaoNeural'
        self.voice_settings['ja']['voice_id'] = 'ja-JP-NanamiNeural'

    def _check_audio_output(self):
        """检查音频输出设备"""
        return os.system("aplay -l") == 0

    def speak(self, text, language=None):
        """文本转语音"""
        if not language:
            language = self.detect_language(text)

        if self.use_edge_tts:
            return asyncio.run(self.edge_tts_speak(text, language))
        else:
            return self.pyttsx3_speak(text, language)

    def pyttsx3_speak(self, text, language):
        """使用pyttsx3执行TTS"""
        if not self.is_engine_ready:
            rospy.logwarn("pyttsx3引擎未就绪，跳过语音播放")
            return False

        try:
            settings = self.voice_settings.get(language, self.voice_settings['en'])
            self.engine.setProperty('voice', settings['voice_id'])
            self.engine.setProperty('rate', settings['rate'])
            self.engine.say(text)
            self.engine.runAndWait()
            return True
        except Exception as e:
            rospy.logerr(f"pyttsx3转换错误: {str(e)}")
            return False

    async def edge_tts_speak(self, text, language):
        """使用Edge-TTS执行TTS"""
        if not self.is_engine_ready:
            rospy.logwarn("Edge-TTS引擎未就绪，跳过语音播放")
            return False

        try:
            # 获取语音设置
            settings = self.voice_settings.get(language, self.voice_settings['en'])
            voice_id = settings['voice_id']

            # 修正 rate 参数，确保是字符串并符合 Edge-TTS 格式
            if isinstance(settings['rate'], int):  # 如果是整数，转换为 Edge-TTS 需要的 "+200%" 格式
                rate = f"+{settings['rate']}%"
            elif isinstance(settings['rate'], str) and settings['rate'].startswith(("+", "-", "0")):
                rate = settings['rate']
            else:
                rate = "+0%"  # 默认语速

            # 执行语音合成
            communicate = edge_tts.Communicate(text=text, voice=voice_id, rate=rate)
            output_file = "/tmp/output.mp3"  # 临时存储生成的音频
            await communicate.save(output_file)

            # 播放音频
            os.system(f"mpg123 {output_file}")
            return True
        except Exception as e:
            rospy.logerr(f"Edge-TTS转换错误: {str(e)}")
            return False

    def detect_language(self, text):
        """简单语言检测（根据内容）"""
        if any('\u4e00' <= char <= '\u9fff' for char in text):
            return 'zh'
        elif any('\u3040' <= char <= '\u30ff' for char in text):
            return 'ja'
        else:
            return 'en'

    def handle_tts_request(self, req):
        """处理 Text To Speech(TTS) 服务请求"""
        success = self.speak(req.text, req.language if req.language else None)
        message = "Success" if success else "Failed (No audio device or error occurred)"
        return TextToSpeechResponse(success=success, message=message)

    def handle_tts_stop(self, req):
        """停止TTS播放"""
        if self.use_edge_tts:
            rospy.logwarn("Edge-TTS 不支持主动停止")
        else:
            self.engine.stop()
        return EmptyResponse()

    def run(self):
        rospy.init_node("speech_generator")
        rospy.Service("/text_to_speech", TextToSpeech, self.handle_tts_request)
        rospy.Service("/stop_tts", Empty, self.handle_tts_stop)
        rospy.loginfo("Speech Generator Node is ready.")
        rospy.spin()

if __name__ == "__main__":
    speech_generator = SpeechGenerator()
    speech_generator.run()
