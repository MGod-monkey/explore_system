#!/bin/bash

# 无人机自主搜索系统 - 快速安装脚本
# 版本: v1.0
# 用途: 一键安装系统依赖、打包程序并创建桌面图标
#
# 使用方法:
#   bash quick_install.sh
#   或
#   ./quick_install.sh
#
# 注意: 请使用 bash 而不是 sh 运行此脚本

set -e  # 遇到错误立即退出

# 检查是否使用bash运行
if [ -z "$BASH_VERSION" ]; then
    echo "错误: 请使用 bash 运行此脚本"
    echo "正确用法: bash quick_install.sh"
    exit 1
fi

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查是否为root用户
check_root() {
    if [ "$EUID" -eq 0 ]; then
        print_error "请不要使用root用户运行此脚本"
        exit 1
    fi
}

# 检查Ubuntu版本
check_ubuntu_version() {
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        if [[ "$ID" != "ubuntu" ]]; then
            print_warning "检测到非Ubuntu系统: $ID"
            print_warning "脚本可能无法正常工作"
        elif [[ "$VERSION_ID" != "20.04" ]]; then
            print_warning "检测到Ubuntu版本: $VERSION_ID"
            print_warning "推荐使用Ubuntu 20.04"
        else
            print_success "检测到Ubuntu 20.04"
        fi
    fi
}

# 检查ROS环境
check_ros_environment() {
    print_info "检查ROS环境..."
    
    if [ -f "/opt/ros/noetic/setup.bash" ]; then
        print_success "找到ROS Noetic"
        source /opt/ros/noetic/setup.bash
    else
        print_error "未找到ROS Noetic，请先安装ROS"
        print_info "安装命令:"
        echo "sudo apt update"
        echo "sudo apt install ros-noetic-desktop-full"
        exit 1
    fi
}

# 安装系统依赖
install_system_dependencies() {
    print_info "安装系统依赖..."
    
    # 更新包列表
    sudo apt update
    
    # 安装基础依赖
    sudo apt install -y \
        python3-pip \
        python3-dev \
        python3-pyqt5 \
        python3-opencv \
        build-essential \
        git
    
    # 安装ROS相关包
    sudo apt install -y \
        ros-noetic-cv-bridge \
        ros-noetic-image-transport \
        ros-noetic-mavros \
        ros-noetic-mavros-extras \
        ros-noetic-realsense2-camera
    
    print_success "系统依赖安装完成"
}

# 安装Python依赖
install_python_dependencies() {
    print_info "安装Python依赖..."

    # 升级pip
    python3 -m pip install --upgrade pip

    # 安装必需的Python包
    pip3 install --user \
        pyinstaller \
        numpy \
        opencv-python \
        psutil \
        PyQt5

    # 确保用户本地bin目录在PATH中
    LOCAL_BIN="$HOME/.local/bin"
    if [ -d "$LOCAL_BIN" ] && [[ ":$PATH:" != *":$LOCAL_BIN:"* ]]; then
        export PATH="$LOCAL_BIN:$PATH"
        print_info "已将 $LOCAL_BIN 添加到PATH"
    fi

    print_success "Python依赖安装完成"
}

# 检查项目文件
check_project_files() {
    print_info "检查项目文件..."

    # 使用兼容sh的方式检查文件
    required_files="start.py dashboard.py topics_subscriber.py topic_logger.py ball_pose_tracker.py build_executable.py install_desktop_icon.sh topics_config.json my_config.rviz"

    missing_files=""

    for file in $required_files; do
        if [ ! -f "$file" ]; then
            missing_files="$missing_files $file"
        fi
    done

    if [ -n "$missing_files" ]; then
        print_error "缺少以下必需文件:"
        for file in $missing_files; do
            echo "  - $file"
        done
        exit 1
    fi

    print_success "所有必需文件都存在"
}

# 打包程序
build_executable() {
    print_info "开始打包程序..."

    # 设置ROS环境
    source /opt/ros/noetic/setup.bash

    # 确保用户本地bin目录在PATH中
    LOCAL_BIN="$HOME/.local/bin"
    if [ -d "$LOCAL_BIN" ] && [[ ":$PATH:" != *":$LOCAL_BIN:"* ]]; then
        export PATH="$LOCAL_BIN:$PATH"
        print_info "已将 $LOCAL_BIN 添加到PATH"
    fi

    # 运行打包脚本
    python3 build_executable.py
    
    if [ $? -eq 0 ]; then
        print_success "程序打包完成"
    else
        print_error "程序打包失败"
        exit 1
    fi
}

# 安装程序
install_program() {
    print_info "安装程序..."
    
    # 检查打包结果
    if [ ! -d "drone_search_system_release" ]; then
        print_error "未找到打包结果目录"
        exit 1
    fi
    
    # 创建安装目录
    INSTALL_DIR="$HOME/drone_search_system"
    
    if [ -d "$INSTALL_DIR" ]; then
        print_warning "安装目录已存在，将覆盖现有安装"
        rm -rf "$INSTALL_DIR"
    fi
    
    # 复制文件
    cp -r drone_search_system_release "$INSTALL_DIR"
    
    # 设置权限
    chmod +x "$INSTALL_DIR/drone_search_system"
    chmod +x "$INSTALL_DIR/run_drone_system.sh"
    chmod +x "$INSTALL_DIR/install_desktop_icon.sh"
    
    print_success "程序安装到: $INSTALL_DIR"
}

# 创建桌面图标
create_desktop_icon() {
    print_info "创建桌面图标..."
    
    INSTALL_DIR="$HOME/drone_search_system"
    
    # 进入安装目录并运行安装脚本
    cd "$INSTALL_DIR"
    ./install_desktop_icon.sh
    
    if [ $? -eq 0 ]; then
        print_success "桌面图标创建完成"
    else
        print_warning "桌面图标创建可能失败，请手动运行安装脚本"
    fi
}

# 清理临时文件
cleanup() {
    print_info "清理临时文件..."
    
    # 清理打包过程中的临时文件
    if [ -d "build_executable" ]; then
        rm -rf build_executable
    fi
    
    if [ -d "build" ]; then
        rm -rf build
    fi
    
    if [ -d "dist" ]; then
        rm -rf dist
    fi
    
    if [ -f "*.spec" ]; then
        rm -f *.spec
    fi
    
    print_success "清理完成"
}

# 显示安装结果
show_installation_result() {
    echo ""
    echo "=========================================="
    print_success "安装完成！"
    echo "=========================================="
    echo ""
    echo "程序安装位置: $HOME/drone_search_system"
    echo ""
    echo "启动方式:"
    echo "1. 在应用程序菜单中搜索 '无人机自主搜索系统'"
    echo "2. 运行命令: $HOME/drone_search_system/run_drone_system.sh"
    echo "3. 双击桌面快捷方式（如果创建了）"
    echo ""
    echo "配置文件位置:"
    echo "- 程序配置: $HOME/drone_search_system/"
    echo "- 截图目录: $HOME/drone_search_system/ball_screenshots/"
    echo "- 日志目录: $HOME/drone_search_system/log/"
    echo ""
    print_info "首次运行可能需要较长时间初始化"
    print_info "确保ROS环境正确配置后再启动程序"
    echo ""
}

# 主函数
main() {
    echo "=========================================="
    echo "    无人机自主搜索系统 - 快速安装脚本"
    echo "=========================================="
    echo ""
    
    # 检查环境
    check_root
    check_ubuntu_version
    check_ros_environment
    check_project_files
    
    # 询问用户是否继续
    echo ""
    read -p "是否继续安装? (y/N): " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_info "安装已取消"
        exit 0
    fi
    
    # 执行安装步骤
    install_system_dependencies
    install_python_dependencies
    build_executable
    install_program
    create_desktop_icon
    cleanup
    show_installation_result
}

# 错误处理
trap 'print_error "安装过程中发生错误，请检查输出信息"; exit 1' ERR

# 运行主函数
main "$@"
