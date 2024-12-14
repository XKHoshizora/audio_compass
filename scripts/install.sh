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
    local package=$1
    case $package in
        "SpeechRecognition")
            package="speech_recognition"
            ;;
        "openai-whisper")
            package="whisper"
            ;;
    esac

    if python3 -c "import $package" &> /dev/null; then
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
    "libopenblas-dev"
    "libopenmpi-dev"
    "libasound-dev"
    "ffmpeg"
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

# 升级pip
print_msg $YELLOW "升级pip..."
python3 -m pip install --upgrade pip

# 安装Python包
print_msg $YELLOW "安装Python包..."
python_deps=(
    "pyttsx3"
    "pyaudio"
    "vosk"
    "SpeechRecognition"
    "scipy"
    "numpy"
)

for dep in "${python_deps[@]}"; do
    if ! check_python_package $dep; then
        print_msg $YELLOW "安装 $dep..."
        python3 -m pip install --no-cache-dir $dep
    else
        print_msg $GREEN "$dep 已安装"
    fi
done

# 安装OpenAI Whisper
print_msg $YELLOW "安装 OpenAI Whisper..."
if ! check_python_package "openai-whisper"; then
    python3 -m pip install --no-cache-dir openai-whisper
fi

# 安装PyTorch
if [[ $(uname -m) == "aarch64" ]]; then
    print_msg $YELLOW "检测到Jetson平台，安装PyTorch..."

    # 设置PyTorch安装URL
    TORCH_URL="https://developer.download.nvidia.cn/compute/redist/jp/v511/pytorch/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl"

    print_msg $YELLOW "设置LD_LIBRARY_PATH..."
    export LD_LIBRARY_PATH=/usr/lib/llvm-8/lib:$LD_LIBRARY_PATH

    print_msg $YELLOW "安装PyTorch..."
    python3 -m pip install --no-cache --upgrade ${TORCH_URL}

    if [ $? -eq 0 ]; then
        print_msg $GREEN "PyTorch安装成功"
    else
        print_msg $RED "PyTorch安装失败"
    fi
else
    print_msg $YELLOW "非Jetson平台，安装标准版PyTorch..."
    python3 -m pip install torch
fi

# 验证PyTorch安装
print_msg $YELLOW "验证PyTorch安装..."
if python3 -c "import torch; print('PyTorch version:', torch.__version__)"; then
    print_msg $GREEN "PyTorch验证成功"
else
    print_msg $RED "PyTorch验证失败"
fi

# 检查音频设备
print_msg $YELLOW "检查音频设备..."
if ! arecord -l &> /dev/null; then
    print_msg $YELLOW "警告: 未检测到录音设备"
else
    print_msg $GREEN "检测到录音设备"
fi

if ! aplay -l &> /dev/null; then
    print_msg $YELLOW "警告: 未检测到播放设备"
else
    print_msg $GREEN "检测到播放设备"
fi

# 最后的验证
print_msg $YELLOW "验证所有安装..."
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

print_msg $GREEN "安装脚本执行完成！"
print_msg $YELLOW "请注意："
print_msg $YELLOW "1. 如果看到pip权限警告，可以忽略"
print_msg $YELLOW "2. 如果音频设备未就绪，可以稍后插入"
print_msg $YELLOW "3. 可能需要运行: export LD_LIBRARY_PATH=/usr/lib/llvm-8/lib:\$LD_LIBRARY_PATH"
print_msg $YELLOW "4. 建议重新启动终端以确保所有更改生效"