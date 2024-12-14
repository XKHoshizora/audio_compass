#!/bin/bash

# 设置颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_msg() {
    local color=$1
    local msg=$2
    echo -e "${color}${msg}${NC}"
}

# 检查命令是否存在
check_command() {
    if ! command -v $1 &> /dev/null; then
        return 1
    fi
    return 0
}

# 检查Python包是否已安装
check_python_package() {
    if python3 -c "import $1" &> /dev/null; then
        return 0
    fi
    return 1
}

# 检查ROS包是否已安装
check_ros_package() {
    if rospack find $1 &> /dev/null; then
        return 0
    fi
    return 1
}

# 确保脚本以root权限运行
if [ "$EUID" -ne 0 ]; then
    print_msg $RED "请使用sudo运行此脚本"
    exit 1
fi

# 检查ROS环境
if ! check_command rosversion; then
    print_msg $RED "未检测到ROS环境，请先安装ROS并source环境"
    exit 1
fi

print_msg $GREEN "开始安装依赖..."

# 更新包列表
print_msg $YELLOW "更新包列表..."
apt-get update

# 安装系统依赖
print_msg $YELLOW "安装系统依赖..."
system_deps=(
    "python3-pip"
    "portaudio19-dev"
    "python3-pyaudio"
    "libopenblas-base"
    "libopenmpi-dev"
    "libasound-dev"
    "ros-noetic-tf2"
    "ros-noetic-tf2-ros"
    "ros-noetic-tf2-geometry-msgs"
    "python3-tf2-geometry-msgs"
)

for dep in "${system_deps[@]}"; do
    if ! dpkg -l | grep -q "^ii  $dep "; then
        print_msg $YELLOW "安装 $dep..."
        apt-get install -y $dep
    else
        print_msg $GREEN "$dep 已安装"
    fi
done

# 安装Python包
print_msg $YELLOW "安装Python包..."
python_deps=(
    "pyttsx3"
    "pyaudio"
    "vosk"
    "SpeechRecognition"
    "scipy"
    "whisper"
    "numpy"
)

for dep in "${python_deps[@]}"; do
    if ! check_python_package $dep; then
        print_msg $YELLOW "安装 $dep..."
        pip3 install $dep
    else
        print_msg $GREEN "$dep 已安装"
    fi
done

# 特殊处理: PyTorch for Jetson
if [[ $(uname -m) == "aarch64" ]]; then
    print_msg $YELLOW "检测到Jetson平台，安装特定版本的PyTorch..."
    if ! check_python_package "torch"; then
        wget https://nvidia.box.com/shared/static/p57jwntv436lfrd78inwl7iml6p13fzh.whl -O torch-1.8.0-cp36-cp36m-linux_aarch64.whl
        pip3 install torch-1.8.0-cp36-cp36m-linux_aarch64.whl
        rm torch-1.8.0-cp36-cp36m-linux_aarch64.whl
    else
        print_msg $GREEN "PyTorch 已安装"
    fi
else
    if ! check_python_package "torch"; then
        print_msg $YELLOW "安装 PyTorch..."
        pip3 install torch
    else
        print_msg $GREEN "PyTorch 已安装"
    fi
fi

# 检查音频设备
print_msg $YELLOW "检查音频设备..."
if ! arecord -l &> /dev/null; then
    print_msg $RED "警告: 未检测到录音设备"
else
    print_msg $GREEN "检测到录音设备"
fi

if ! aplay -l &> /dev/null; then
    print_msg $RED "警告: 未检测到播放设备"
else
    print_msg $GREEN "检测到播放设备"
fi

# 最后的检查
print_msg $YELLOW "验证安装..."
all_deps_installed=true

for dep in "${python_deps[@]}"; do
    if ! check_python_package $dep; then
        print_msg $RED "警告: $dep 安装失败"
        all_deps_installed=false
    fi
done

if [ "$all_deps_installed" = true ]; then
    print_msg $GREEN "所有依赖安装成功！"
else
    print_msg $RED "部分依赖安装失败，请检查上述错误信息"
fi

# 创建一个标记文件表示已经运行过安装脚本
touch ~/.audio_compass_deps_installed

print_msg $GREEN "安装脚本执行完成！"
print_msg $YELLOW "请注意：某些功能可能需要重新启动终端才能生效"