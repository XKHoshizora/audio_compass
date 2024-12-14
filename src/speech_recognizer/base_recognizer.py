import rospy
import pyaudio

class BaseRecognizer:
    """语音识别器的基类"""

    def __init__(self):
        # 检查音频输入设备
        if not self._check_audio_input():
            rospy.logerr("未检测到麦克风设备")
            raise RuntimeError("未检测到麦克风设备")

        self.p = pyaudio.PyAudio()

    def _check_audio_input(self):
        """检查是否有可用的麦克风设备"""
        try:
            p = pyaudio.PyAudio()
            input_device_count = p.get_host_api_info_by_index(0)['deviceCount']

            # 检查是否存在输入设备
            found_input = False
            for i in range(input_device_count):
                device_info = p.get_device_info_by_index(i)
                if device_info['maxInputChannels'] > 0:
                    rospy.loginfo(f"找到麦克风设备: {device_info['name']}")
                    found_input = True
                    break

            p.terminate()
            return found_input
        except Exception as e:
            rospy.logerr(f"检查麦克风设备时出错: {e}")
            return False

    def cleanup(self):
        if hasattr(self, 'p'):
            self.p.terminate()