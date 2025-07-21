#!/bin/bash

# 无人机自主搜索系统简化打包脚本
# 适用于Ubuntu 20.04 + ROS Noetic

echo "=== 无人机自主搜索系统打包工具 ==="
echo "正在准备打包环境..."

# 检查必要的依赖
echo "检查依赖..."

# 检查PyInstaller
if ! python3 -c "import PyInstaller" 2>/dev/null; then
    echo "安装PyInstaller..."
    pip3 install pyinstaller
fi

# 确保PyInstaller在PATH中
export PATH="$HOME/.local/bin:$PATH"

# 验证PyInstaller可用性
if ! command -v pyinstaller &> /dev/null; then
    echo "尝试使用python3 -m PyInstaller..."
    PYINSTALLER_CMD="python3 -m PyInstaller"
else
    PYINSTALLER_CMD="pyinstaller"
fi

# 检查其他依赖
pip3 install numpy opencv-python psutil

# 设置ROS环境
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
    echo "✅ ROS环境已加载"
else
    echo "❌ 未找到ROS环境"
    exit 1
fi

# 创建构建目录
BUILD_DIR="build_package"
if [ -d "$BUILD_DIR" ]; then
    rm -rf "$BUILD_DIR"
fi
mkdir -p "$BUILD_DIR"

echo "复制文件到构建目录..."

# 复制主要文件
cp start.py "$BUILD_DIR/"
cp dashboard.py "$BUILD_DIR/" 2>/dev/null || echo "⚠️  dashboard.py 不存在"
cp topics_subscriber.py "$BUILD_DIR/" 2>/dev/null || echo "⚠️  topics_subscriber.py 不存在"
cp topic_logger.py "$BUILD_DIR/" 2>/dev/null || echo "⚠️  topic_logger.py 不存在"
cp images_rc.py "$BUILD_DIR/" 2>/dev/null || echo "⚠️  images_rc.py 不存在"
cp topics_config.json "$BUILD_DIR/" 2>/dev/null || echo "⚠️  topics_config.json 不存在"
cp my_config.rviz "$BUILD_DIR/" 2>/dev/null || echo "⚠️  my_config.rviz 不存在"
cp logo.png "$BUILD_DIR/" 2>/dev/null || echo "⚠️  logo.png 不存在"

# 复制资源目录
if [ -d "resource" ]; then
    cp -r resource "$BUILD_DIR/"
    echo "✅ 复制资源目录"
else
    echo "⚠️  resource 目录不存在"
fi

# 复制截图目录
if [ -d "ball_screenshots" ]; then
    cp -r ball_screenshots "$BUILD_DIR/"
    echo "✅ 复制截图目录"
else
    mkdir -p "$BUILD_DIR/ball_screenshots"
    echo "✅ 创建截图目录"
fi

# 进入构建目录
cd "$BUILD_DIR"

echo "开始打包..."

# 使用PyInstaller打包
$PYINSTALLER_CMD --onefile \
    --name="drone_search_system" \
    --add-data="topics_config.json:." \
    --add-data="my_config.rviz:." \
    --add-data="logo.png:." \
    --add-data="resource:resource" \
    --add-data="ball_screenshots:ball_screenshots" \
    --exclude-module="PySide2" \
    --exclude-module="PySide6" \
    --exclude-module="tkinter" \
    --hidden-import="cv2" \
    --hidden-import="numpy" \
    --hidden-import="psutil" \
    --hidden-import="rospy" \
    --hidden-import="roslib" \
    --hidden-import="geometry_msgs.msg" \
    --hidden-import="sensor_msgs.msg" \
    --hidden-import="visualization_msgs.msg" \
    --hidden-import="std_msgs.msg" \
    --hidden-import="rviz" \
    --hidden-import="python_qt_binding.QtCore" \
    --hidden-import="python_qt_binding.QtGui" \
    --hidden-import="python_qt_binding.QtWidgets" \
    --hidden-import="PyQt5.QtCore" \
    --hidden-import="PyQt5.QtGui" \
    --hidden-import="PyQt5.QtWidgets" \
    --hidden-import="dashboard" \
    --hidden-import="topics_subscriber" \
    --hidden-import="topic_logger" \
    --hidden-import="images_rc" \
    --console \
    start.py

if [ $? -eq 0 ]; then
    echo "✅ 打包成功!"
    
    # 创建最终发布目录
    cd ..
    PACKAGE_DIR="drone_search_system_release"
    if [ -d "$PACKAGE_DIR" ]; then
        rm -rf "$PACKAGE_DIR"
    fi
    mkdir -p "$PACKAGE_DIR"
    
    # 复制可执行文件
    cp "$BUILD_DIR/dist/drone_search_system" "$PACKAGE_DIR/"
    chmod +x "$PACKAGE_DIR/drone_search_system"
    
    # 复制配置文件
    cp "$BUILD_DIR/topics_config.json" "$PACKAGE_DIR/" 2>/dev/null
    cp "$BUILD_DIR/my_config.rviz" "$PACKAGE_DIR/" 2>/dev/null
    cp "$BUILD_DIR/logo.png" "$PACKAGE_DIR/" 2>/dev/null
    
    # 复制资源目录
    if [ -d "$BUILD_DIR/resource" ]; then
        cp -r "$BUILD_DIR/resource" "$PACKAGE_DIR/"
    fi
    
    # 复制截图目录
    if [ -d "$BUILD_DIR/ball_screenshots" ]; then
        cp -r "$BUILD_DIR/ball_screenshots" "$PACKAGE_DIR/"
    fi
    
    # 创建启动脚本
    cat > "$PACKAGE_DIR/run_drone_system.sh" << 'EOF'
#!/bin/bash

# 无人机自主搜索系统启动脚本

echo "=== 无人机自主搜索系统 ==="

# 设置ROS环境
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
    echo "✅ ROS环境已加载"
else
    echo "❌ 未找到ROS环境，请确保ROS Noetic已正确安装"
    exit 1
fi

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

# 检查roscore
if ! pgrep -x "roscore" > /dev/null; then
    echo "启动roscore..."
    roscore &
    sleep 3
fi

echo "启动无人机自主搜索系统..."
./drone_search_system

echo "程序已退出"
EOF
    
    chmod +x "$PACKAGE_DIR/run_drone_system.sh"
    
    # 创建README
    cat > "$PACKAGE_DIR/README.md" << 'EOF'
# 无人机自主搜索系统

## 系统要求
- Ubuntu 20.04
- ROS Noetic
- Python 3.8+

## 运行方法
```bash
./run_drone_system.sh
```

## 文件说明
- `drone_search_system`: 主程序可执行文件
- `run_drone_system.sh`: 启动脚本
- `my_config.rviz`: RViz配置文件
- `topics_config.json`: 话题配置文件
- `resource/`: 资源文件目录
- `ball_screenshots/`: 截图保存目录

## 注意事项
1. 首次运行可能需要输入管理员密码设置串口权限
2. 确保ROS Noetic环境已正确安装
3. 如遇问题，请检查ROS环境变量设置

## 故障排除
- 如果程序无法启动，请检查ROS环境: `echo $ROS_PACKAGE_PATH`
- 如果缺少依赖，请安装: `sudo apt install ros-noetic-desktop-full`
- 权限问题: `chmod +x drone_search_system run_drone_system.sh`
EOF
    
    echo ""
    echo "🎉 打包完成!"
    echo "📦 发布包位置: $PACKAGE_DIR"
    echo ""
    echo "运行方法:"
    echo "  cd $PACKAGE_DIR"
    echo "  ./run_drone_system.sh"
    echo ""
    echo "📋 包含文件:"
    ls -la "$PACKAGE_DIR"
    
else
    echo "❌ 打包失败!"
    echo "请检查错误信息并重试"
    exit 1
fi
