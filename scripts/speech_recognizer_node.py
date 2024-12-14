#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from speech_recognizer import GoogleSpeechRecognizer, VoskSpeechRecognizer, WhisperSpeechRecognizer


class SpeechRecognizer:
    """管理不同语音识别系统的ROS节点"""
    
    # 支持的识别器类型
    SUPPORTED_RECOGNIZERS = {
        'google': GoogleSpeechRecognizer,
        'vosk': VoskSpeechRecognizer,
        'whisper': WhisperSpeechRecognizer
    }
    
    # 支持的语言列表
    SUPPORTED_LANGUAGES = ['en-US', 'ja-JP', 'zh-CN']

    # 默认的触发词模式
    DEFAULT_TRIGGER_PATTERNS = [
        r'たっくん',
        r'タックン',
        r'タッくん',
        r'たッくん',
        r'るみちゃん',
        r'ルミちゃん',
        r'たっ くん',
        r'るみ ちゃん',
        # 可以根据需要添加更多...
    ]
    
    def __init__(self):
        """初始化语音识别节点"""
        rospy.init_node('speech_recognizer', anonymous=True)
        
        # 从参数服务器获取参数
        self.recognizer_type = rospy.get_param('~recognizer_type', 'google')
        self.language = rospy.get_param('~language', 'en-US')

        # 获取自定义触发词列表（如果有的话）
        custom_triggers = rospy.get_param('~trigger_patterns', None)
        self.trigger_patterns = custom_triggers if custom_triggers else self.DEFAULT_TRIGGER_PATTERNS
        
        # 获取Whisper特定的参数（如果使用Whisper的话）
        self.whisper_model = rospy.get_param('~whisper_model', 'tiny')
        
        # 验证参数
        self._validate_parameters()
        
        # 初始化选定的语音识别系统
        self._initialize_recognizer()
        
    def _validate_parameters(self):
        """验证launch文件中的参数"""
        if self.recognizer_type not in self.SUPPORTED_RECOGNIZERS:
            rospy.logerr(f"不支持的识别器类型: {self.recognizer_type}")
            rospy.logerr(f"支持的类型包括: {list(self.SUPPORTED_RECOGNIZERS.keys())}")
            rospy.logerr("默认使用 'google'")
            self.recognizer_type = 'google'
            
        if self.language not in self.SUPPORTED_LANGUAGES:
            rospy.logerr(f"不支持的语言: {self.language}")
            rospy.logerr(f"支持的语言包括: {self.SUPPORTED_LANGUAGES}")
            rospy.logerr("默认使用 'en-US'")
            self.language = 'en-US'
            
    def _initialize_recognizer(self):
        """初始化选定的语音识别系统"""
        try:
            recognizer_class = self.SUPPORTED_RECOGNIZERS[self.recognizer_type]

            # 初始化识别器，传入必要的参数
            init_params = {
                'language': self.language,
                'trigger_patterns': self.trigger_patterns
            }

            # 如果是Whisper识别器，添加额外的参数
            if self.recognizer_type == 'whisper':
                init_params['model_name'] = self.whisper_model

            self.recognizer = recognizer_class(**init_params)

            rospy.loginfo(f"已初始化 {self.recognizer_type} 语音识别器")
            rospy.loginfo(f"使用语言: {self.language}")
            rospy.loginfo(f"触发词数量: {len(self.trigger_patterns)}")

            if self.recognizer_type == 'whisper':
                rospy.loginfo(f"Whisper模型: {self.whisper_model}")
                
        except Exception as e:
            rospy.logerr(f"初始化 {self.recognizer_type} 识别器失败: {str(e)}")
            raise
            
    def run(self):
        """启动语音识别系统"""
        try:
            self.recognizer.start()
        except Exception as e:
            rospy.logerr(f"运行语音识别器时出错: {str(e)}")
            self.cleanup()
            
    def cleanup(self):
        """关闭时清理资源"""
        if hasattr(self, 'recognizer'):
            self.recognizer.cleanup()
        rospy.loginfo("语音识别节点已关闭")


if __name__ == '__main__':
    try:
        recognizer = SpeechRecognizer()
        recognizer.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"语音识别系统启动失败: {str(e)}")