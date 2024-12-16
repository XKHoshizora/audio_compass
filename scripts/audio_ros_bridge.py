#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading
import rospy
import actionlib
import tf.transformations as tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from audio_compass.msg import SpeechDirection
from audio_compass.srv import TextToSpeech, TextToSpeechResponse
from wp_nav_controller.srv import NavigationCommand, NavigationCommandResponse
from wp_nav_controller.srv import SequenceCommand, SequenceCommandResponse


class AudioRosBridge:
    def __init__(self):
        rospy.init_node('audio_ros_bridge', anonymous=True)

        # 初始化 move_base
        # 创建 move_base 的 Action Client
        self.mb_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("等待 move_base 动作服务器...")
        self.mb_client.wait_for_server()
        rospy.loginfo("move_base 动作服务器已连接.")

        # 初始化导航控制服务
        # 单点导航服务
        # rospy.wait_for_service('/wp_nav_controller/navigation')
        # self.nav_client = rospy.ServiceProxy('/wp_nav_controller/navigation', NavigationCommand)

        # 序列导航服务
        rospy.wait_for_service('/wp_nav_controller/sequence')
        self.seq_client = rospy.ServiceProxy('/wp_nav_controller/sequence', SequenceCommand)

        # 初始化语音合成服务
        self.tts_thread = None
        rospy.wait_for_service('text_to_speech')
        self.tts_client = rospy.ServiceProxy('text_to_speech', TextToSpeech)

        # 订阅语音识别结果
        self.sub = rospy.Subscriber('speech_direction', SpeechDirection, self.speech_recognition_callback)

        rospy.loginfo("Audio ROS Bridge 初始化完成")

    def async_speak(self, say):
        """异步执行语音合成"""
        if self.tts_thread and self.tts_thread.is_alive():
            # 如果上一个语音还在播放，等待其完成，可能会导致导航控制流程有短暂延迟
            self.tts_thread.join()

        # 创建并启动新的语音播放线程
        self.tts_thread = threading.Thread(
            target=self._speak_thread,
            args=(say,)
        )
        self.tts_thread.daemon = True
        self.tts_thread.start()

    def _speak_thread(self, say):
        """语音合成线程函数"""
        try:
            response = self.tts_client(text=say)

            if response.success:
                rospy.loginfo(f"语音合成成功: {say}")
                return True
            else:
                rospy.logerr(f"语音合成失败: {response.message}")
                return False

        except rospy.ServiceException as e:
            rospy.logerr(f"TTS 服务调用失败: {e}")
            return False

    def speech_recognition_callback(self, data):
        """处理语音识别结果"""
        # 识别结果
        text = data.text
        # 方向
        direction = data.yaw

        # 处理识别结果
        rospy.loginfo(f"识别结果: {text}, 方向: {math.degrees(direction)}")

        # 处理导航请求
        self.navigation_process(direction)

    def pause_navigation(self):
        """暂停导航"""
        try:
            self.async_speak("暂停导航")

            # response = self.nav_client(command='pause')
            response = self.seq_client(command='pause')

            if response.success:
                rospy.loginfo("导航已暂停")
                return True
            else:
                rospy.logerr(f"暂停导航失败：{response.message}")
                self.async_speak("暂停导航失败")
                return False

        except rospy.ServiceException as e:
            rospy.logerr(f"暂停导航服务调用失败: {e}")
            return False

    def resume_navigation(self):
        """恢复导航"""
        try:
            self.async_speak("恢复导航")

            # response = self.nav_client(command='resume')
            response = self.seq_client(command='resume')

            if response.success:
                rospy.loginfo("导航已恢复")
                return True
            else:
                rospy.logerr(f"恢复导航失败：{response.message}")
                self.async_speak("恢复导航失败")
                return False

        except rospy.ServiceException as e:
            rospy.logerr(f"恢复导航服务调用失败: {e}")
            return False

    def navigation_process(self, angle_rad):
        """控制机器人到指定目标点和位姿"""
        try:
            # 检查tf变换是否就绪
            tf_listener = tf.TransformListener()
            if not tf_listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(5.0)):
                rospy.logerr("无法获取从map到base_link的转换，请确认机器人位置已初始化")
                self.async_speak("请先设置机器人初始位置")
                return False

            # 暂停导航
            self.pause_navigation()

            # 将角度从度转换为弧度
            # angle_rad = math.radians(angle_deg)

            # 创建目标姿态
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "base_link"  # 基于机器人本地坐标系
            goal.target_pose.header.stamp = rospy.Time.now()

            # 设置目标点
            goal.target_pose.pose.position.x = 0.0
            goal.target_pose.pose.position.y = 0.0
            goal.target_pose.pose.position.z = 0.0

            # 设置位姿，使用四元数
            quaternion = tf.quaternion_from_euler(0, 0, angle_rad)  # Roll, Pitch, Yaw
            goal.target_pose.pose.orientation.x = quaternion[0]
            goal.target_pose.pose.orientation.y = quaternion[1]
            goal.target_pose.pose.orientation.z = quaternion[2]
            goal.target_pose.pose.orientation.w = quaternion[3]

            # 发送目标
            rospy.loginfo(f"发送导航目标: {math.degrees(angle_rad)} 度")
            self.async_speak("开始转向")
            self.mb_client.send_goal(goal)

            # 等待结果
            self.mb_client.wait_for_result()
            result = self.mb_client.get_result()
            if result:
                rospy.loginfo("导航成功")
                self.async_speak("导航成功")

                # 执行一些操作并等待完成（如购买东西、播放视频等）
                rospy.sleep(5)
            else:
                rospy.logerr("导航失败")
                self.async_speak("导航失败")

            # 恢复导航
            self.resume_navigation()

        except Exception as e:
            rospy.logerr(f"导航过程出错: {e}")
            return False

    def run(self):
        """运行节点"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            self.cleanup()

    def cleanup(self):
        """清理资源"""
        rospy.loginfo("Audio ROS Bridge 节点已关闭")


if __name__ == '__main__':
    try:
        bridge = AudioRosBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Audio ROS Bridge 启动失败: {str(e)}")
