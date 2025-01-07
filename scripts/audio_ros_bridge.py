#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading
import rospy
import actionlib
from tf import transformations
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from audio_compass.msg import SpeechDirection
from audio_compass.srv import TextToSpeech
from wp_nav_controller.srv import NavigationCommand
from wp_nav_controller.srv import SequenceCommand


class AudioRosBridge:
    def __init__(self):
        rospy.init_node('audio_ros_bridge', anonymous=True)

        # 初始化 move_base
        # 创建 move_base 的 Action Client
        self.mb_client = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction)
        rospy.loginfo("等待 move_base 动作服务器...")
        self.mb_client.wait_for_server()
        rospy.loginfo("move_base 动作服务器已连接.")

        # 初始化导航控制服务
        # 单点导航服务
        # rospy.wait_for_service('/wp_nav_controller/navigation')
        # self.nav_client = rospy.ServiceProxy('/wp_nav_controller/navigation', NavigationCommand)

        # 序列导航服务
        rospy.wait_for_service('/wp_nav_controller/sequence')
        self.seq_client = rospy.ServiceProxy(
            '/wp_nav_controller/sequence', SequenceCommand)

        # 初始化语音合成服务
        self.tts_thread = None
        rospy.wait_for_service('text_to_speech')
        self.tts_client = rospy.ServiceProxy('text_to_speech', TextToSpeech)

        # 订阅语音识别结果
        self.sub = rospy.Subscriber(
            'speech_direction', SpeechDirection, self.speech_recognition_callback)

        self.tts_lock = threading.Lock()
        self.tts_thread = None

        rospy.loginfo("Audio ROS Bridge 初始化完成")

    def async_speak(self, say):
        """异步执行语音合成"""
        with self.tts_lock:
            if self.tts_thread and self.tts_thread.is_alive():
                try:
                    self.tts_thread.join(timeout=1.0)  # 设置超时时间
                    if self.tts_thread.is_alive():
                        rospy.logwarn("上一个TTS任务未完成，跳过当前语音")
                        return False
                except Exception as e:
                    rospy.logerr(f"等待TTS线程时出错: {e}")
                    return False

            # 创建并启动新的语音播放线程
            self.tts_thread = threading.Thread(
                target=self._speak_thread,
                args=(say,),
                daemon=True
            )
            self.tts_thread.start()
            return True

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
            # self.async_speak("暂停导航")
            self.async_speak("一時停止します")

            # response = self.nav_client(command='pause')
            response = self.seq_client(command='pause')

            if response.success:
                rospy.loginfo("导航已暂停")
                return True
            else:
                rospy.logerr(f"暂停导航失败：{response.message}")
                # self.async_speak("暂停导航失败")
                self.async_speak("一時停止に失敗しました")
                return False

        except rospy.ServiceException as e:
            rospy.logerr(f"暂停导航服务调用失败: {e}")
            return False

    def resume_navigation(self):
        """恢复导航"""
        try:
            # self.async_speak("恢复导航")
            self.async_speak("ナビゲーションを再開します")

            # response = self.nav_client(command='resume')
            response = self.seq_client(command='resume')

            if response.success:
                rospy.loginfo("导航已恢复")
                # self.async_speak("导航已恢复")
                self.async_speak("ナビゲーションを再開しました")
                return True
            else:
                rospy.logerr(f"恢复导航失败：{response.message}")
                # self.async_speak("恢复导航失败")
                self.async_speak("ナビゲーションの再開に失敗しました")
                return False

        except rospy.ServiceException as e:
            rospy.logerr(f"恢复导航服务调用失败: {e}")
            return False

    def navigation_process(self, angle_rad):
        """控制机器人到指定目标点和位姿"""

        # 将角度从度转换为弧度（如有需要）
        # angle_rad = math.radians(angle_deg)

        navigation_paused = False  # 标志导航是否已暂停

        try:
            # 尝试暂停导航
            if not self.pause_navigation():
                rospy.logerr("暂停导航失败，无法执行后续操作")
                # self.async_speak("暂停导航失败，无法执行后续操作")
                self.async_speak("ナビゲーションの一時停止に失敗したため、後続の操作を実行できません")
                return False

            navigation_paused = True  # 标记导航已暂停

            # 创建目标姿态
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "base_link"  # 基于机器人本地坐标系
            goal.target_pose.header.stamp = rospy.Time.now()

            # 设置当前位置为目标点（基于机器人本地坐标系时使用）
            goal.target_pose.pose.position.x = 0.0
            goal.target_pose.pose.position.y = 0.0
            goal.target_pose.pose.position.z = 0.0

            # 设置位姿，使用四元数
            target_quaternion = transformations.quaternion_from_euler(0, 0, angle_rad)  # Roll, Pitch, Yaw（基于机器人本地坐标系时使用）
            goal.target_pose.pose.orientation.x = target_quaternion[0]
            goal.target_pose.pose.orientation.y = target_quaternion[1]
            goal.target_pose.pose.orientation.z = target_quaternion[2]
            goal.target_pose.pose.orientation.w = target_quaternion[3]

            # 发送目标
            rospy.loginfo(f"发送导航目标: {math.degrees(angle_rad)} 度")
            # self.async_speak("开始转向")
            self.async_speak("回転します")
            self.mb_client.send_goal(goal)

            # 等待导航完成并获取结果
            self.mb_client.wait_for_result()
            nav_result = self.mb_client.get_result()

            if nav_result == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("导航成功")
                # self.async_speak("导航成功")
                self.async_speak("ナビゲーション成功しました")

                # 执行一些操作并等待其完成（如购买东西、播放视频等）
                rospy.sleep(5)

                return True  # 此处的 return 会被 finally 语句覆盖
            else:
                rospy.logerr("导航失败，状态码: {nav_result}")
                # self.async_speak("导航失败")
                self.async_speak("ナビゲーション失敗しました")
                return False

        except rospy.ServiceException as e:
            rospy.logerr(f"audio_ros_bridge 节点中出现异常: {e}")
            return False
        except Exception as e:
            rospy.logerr(f"audio_ros_bridge 节点中出现未预期异常: {e}")
            return False

        # finally 块总是会执行，即使在 try 块中发生了异常或已执行过 return 语句
        finally:
            # 确保在必要时恢复导航
            if navigation_paused:
                if self.resume_navigation():
                    rospy.loginfo("导航已恢复")
                    self.async_speak("导航已恢复")
                else:
                    rospy.logerr("恢复导航失败")
                    self.async_speak("导航恢复失败，请检查系统状态")
            # 这里没有return语句，所以不会覆盖 finally 块之前的返回值

    def run(self):
        """运行节点"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            self.cleanup()

    def cleanup(self):
        """清理资源"""
        with self.tts_lock:
            if self.tts_thread and self.tts_thread.is_alive():
                self.tts_thread.join(timeout=2.0)  # 等待最多2秒
        rospy.loginfo("Audio ROS Bridge 节点已关闭")


if __name__ == '__main__':
    try:
        bridge = AudioRosBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Audio ROS Bridge 启动失败: {str(e)}")
