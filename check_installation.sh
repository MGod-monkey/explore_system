#!/bin/bash

# 无人机自主搜索系统 - 安装检查脚本
# 用于验证系统安装是否正确

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印函数
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[✓]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[!]${NC} $1"
}

print_error() {
    echo -e "${RED}[✗]${NC} $1"
}

# 检查计数器
total_checks=0
passed_checks=0

# 执行检查
run_check() {
    local description="$1"
    local command="$2"
    
    total_checks=$((total_checks + 1))
    print_info "检查: $description"
    
    if eval "$command" >/dev/null 2>&1; then
        print_success "$description"
        passed_checks=$((passed_checks + 1))
        return 0
    else
        print_error "$description"
        return 1
    fi
}

echo "=========================================="
echo "    无人机自主搜索系统 - 安装检查"
echo "=========================================="
echo ""

# 1. 检查ROS环境
print_info "检查ROS环境..."
run_check "ROS Noetic 安装" "[ -f /opt/ros/noetic/setup.bash ]"
run_check "ROS 环境变量" "[ ! -z \"\$ROS_DISTRO\" ] || source /opt/ros/noetic/setup.bash"

# 2. 检查Python依赖
print_info "检查Python依赖..."
run_check "Python 3" "python3 --version"
run_check "pip3" "pip3 --version"
run_check "PyQt5" "python3 -c 'import PyQt5'"
run_check "OpenCV" "python3 -c 'import cv2'"
run_check "NumPy" "python3 -c 'import numpy'"
run_check "psutil" "python3 -c 'import psutil'"

# 3. 检查ROS包
print_info "检查ROS包..."
source /opt/ros/noetic/setup.bash 2>/dev/null || true
run_check "cv_bridge" "rospack find cv_bridge"
run_check "image_transport" "rospack find image_transport"
run_check "mavros" "rospack find mavros"

# 4. 检查程序文件
print_info "检查程序安装..."
INSTALL_DIR="$HOME/drone_search_system"

run_check "安装目录存在" "[ -d \"$INSTALL_DIR\" ]"
run_check "主程序可执行文件" "[ -x \"$INSTALL_DIR/drone_search_system\" ]"
run_check "启动脚本" "[ -x \"$INSTALL_DIR/run_drone_system.sh\" ]"
run_check "配置文件" "[ -f \"$INSTALL_DIR/topics_config.json\" ]"
run_check "RViz配置" "[ -f \"$INSTALL_DIR/my_config.rviz\" ]"

# 5. 检查桌面图标
print_info "检查桌面集成..."
run_check "应用程序图标" "[ -f \"$HOME/.local/share/applications/drone_search_system.desktop\" ]"

# 6. 检查目录权限
print_info "检查目录权限..."
run_check "安装目录可读" "[ -r \"$INSTALL_DIR\" ]"
run_check "安装目录可写" "[ -w \"$INSTALL_DIR\" ]"

# 7. 检查数据目录
print_info "检查数据目录..."
run_check "截图目录" "[ -d \"$INSTALL_DIR/ball_screenshots\" ]"
run_check "日志目录" "[ -d \"$INSTALL_DIR/log\" ]"

# 8. 检查系统工具
print_info "检查系统工具..."
run_check "roscore 可用" "which roscore"
run_check "rostopic 可用" "which rostopic"

echo ""
echo "=========================================="
echo "           检查结果汇总"
echo "=========================================="
echo ""

if [ $passed_checks -eq $total_checks ]; then
    print_success "所有检查通过 ($passed_checks/$total_checks)"
    echo ""
    echo "🎉 系统安装正确！可以正常使用。"
    echo ""
    echo "启动方式:"
    echo "1. 应用程序菜单 -> 无人机自主搜索系统"
    echo "2. 命令行: $INSTALL_DIR/run_drone_system.sh"
    echo ""
else
    failed_checks=$((total_checks - passed_checks))
    print_warning "检查通过: $passed_checks/$total_checks (失败: $failed_checks)"
    echo ""
    echo "⚠️  发现问题，建议："
    echo ""
    
    if [ $passed_checks -lt $((total_checks * 3 / 4)) ]; then
        echo "1. 重新运行安装脚本: ./quick_install.sh"
        echo "2. 检查系统依赖是否完整安装"
        echo "3. 确保ROS环境正确配置"
    else
        echo "1. 检查具体失败的项目"
        echo "2. 手动修复相关问题"
        echo "3. 大部分功能应该可以正常使用"
    fi
    echo ""
fi

# 额外的使用提示
echo "=========================================="
echo "           使用提示"
echo "=========================================="
echo ""
echo "📋 重要提醒:"
echo "• 首次启动可能需要较长时间"
echo "• 确保摄像头和传感器正确连接"
echo "• 运行前确保 roscore 已启动"
echo ""
echo "🔧 如果遇到问题:"
echo "• 查看日志: $INSTALL_DIR/log/"
echo "• 检查ROS话题: rostopic list"
echo "• 重启ROS: killall roscore && roscore &"
echo ""
echo "📖 详细文档: 打包安装说明.md"
echo ""

exit 0
