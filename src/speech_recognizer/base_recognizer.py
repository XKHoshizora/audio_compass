#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Speech Recognizer Base Class"""
import rospy
import pyaudio
import tkinter as tk
from tkinter import scrolledtext
from threading import Lock, Thread
import datetime


class BaseRecognizer:
    """语音识别器的基类"""

    def __init__(self):
        # 窗口相关参数和初始化
        self.use_window = rospy.get_param(
            '~use_window', False)  # 从ROS参数获取是否使用窗口
        self.window = None
        self.text_area = None
        self.log_lock = Lock()

        if self.use_window:
            # 在单独的线程中运行GUI
            self.gui_thread = Thread(target=self._setup_window)
            self.gui_thread.daemon = True
            self.gui_thread.start()

        # 检查音频输入设备
        if not self._check_audio_input():
            self.log_err("未检测到麦克风设备")
            raise RuntimeError("未检测到麦克风设备")

        self.p = pyaudio.PyAudio()

    def _setup_window(self):
        """设置GUI窗口"""
        self.window = tk.Tk()
        self.window.title("语音识别日志")
        self.window.geometry("600x400")

        # 创建文本区域
        self.text_area = scrolledtext.ScrolledText(self.window, width=70, height=20)
        self.text_area.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)

        # 配置标签颜色
        self.text_area.tag_config("INFO", foreground="black")
        self.text_area.tag_config("WARN", foreground="orange")
        self.text_area.tag_config("ERROR", foreground="red")
        self.text_area.tag_config("SPEECH", foreground="blue")

        # 启动主循环
        self.window.mainloop()

    def _update_text_area(self, message, level, speech):
        """在主线程中更新文本区域"""
        try:
            self.text_area.insert(tk.END, message + "\n")
            if speech:
                self.text_area.tag_add("SPEECH", "end-2c linestart", "end-1c")
            else:
                self.text_area.tag_add(level, "end-2c linestart", "end-1c")
            self.text_area.see(tk.END)
        except Exception:
            pass

    def _log(self, message, level="INFO", speech=False):
        """内部日志记录函数"""
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        formatted_msg = f"[{timestamp}] {message}"

        # GUI日志
        if self.use_window and self.text_area and self.window:
            try:
                with self.log_lock:
                    self.window.after(0, self._update_text_area, formatted_msg, level, speech)
            except Exception as e:
                pass

    def log_err(self, message, speech=False):
        """记录错误日志"""
        rospy.logerr(message)
        self._log(message, "ERROR", speech)

    def log_warn(self, message, speech=False):
        """记录警告日志"""
        rospy.logwarn(message)
        self._log(message, "WARN", speech)

    def log_info(self, message, speech=False):
        """记录信息日志"""
        rospy.loginfo(message)
        self._log(message, "INFO", speech)

    def log_speech(self, message, speech=True):
        """记录语音识别结果"""
        rospy.loginfo(message)
        self._log(message, "INFO", speech)

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
                    self.log_info(f"找到麦克风设备: {device_info['name']}")
                    found_input = True
                    break

            p.terminate()
            return found_input
        except Exception as e:
            self.log_err(f"检查麦克风设备时出错: {e}")
            return False

    def cleanup(self):
        if hasattr(self, 'p'):
            self.p.terminate()
        if self.window:
            try:
                self.window.quit()
                self.window.destroy()
            except:
                pass