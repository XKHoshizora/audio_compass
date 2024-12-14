cmake_minimum_required(VERSION 3.0.2)
project(audio_compass)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    actionlib
    tf2
    tf2_ros
    std_msgs
    geometry_msgs
    tf2_geometry_msgs
    move_base_msgs
    message_generation
    wp_nav_controller
)

# 添加消息文件
add_message_files(
    FILES
    SpeechDirection.msg
)

# 添加服务文件
add_service_files(
    FILES
    TextToSpeech.srv
)

catkin_python_setup()

# 生成消息
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

# 确保消息生成在其他target之前
catkin_package(
    CATKIN_DEPENDS roscpp rospy actionlib tf2 tf2_ros std_msgs geometry_msgs move_base_msgs message_generation message_runtime wp_nav_controller
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
    scripts/speech_recognizer_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
    scripts/speech_generator_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
    scripts/audio_ros_bridge.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)