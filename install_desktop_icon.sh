#!/bin/bash

# 无人机自主搜索系统桌面图标安装脚本

echo "=== 无人机自主搜索系统桌面图标安装 ==="

# 获取当前用户和路径
CURRENT_USER=$(whoami)
CURRENT_DIR=$(pwd)
RELEASE_DIR="$CURRENT_DIR/drone_search_system_release"

echo "当前用户: $CURRENT_USER"
echo "程序路径: $RELEASE_DIR"

# 检查发布包是否存在
if [ ! -d "$RELEASE_DIR" ]; then
    echo "❌ 错误: 未找到发布包目录 $RELEASE_DIR"
    echo "请确保在包含 drone_search_system_release 目录的位置运行此脚本"
    exit 1
fi

# 检查可执行文件是否存在
if [ ! -f "$RELEASE_DIR/drone_search_system" ]; then
    echo "❌ 错误: 未找到可执行文件 $RELEASE_DIR/drone_search_system"
    exit 1
fi

# 检查启动脚本是否存在
if [ ! -f "$RELEASE_DIR/run_drone_system.sh" ]; then
    echo "❌ 错误: 未找到启动脚本 $RELEASE_DIR/run_drone_system.sh"
    exit 1
fi

# 检查图标文件是否存在
if [ ! -f "$RELEASE_DIR/logo.png" ]; then
    echo "⚠️  警告: 未找到图标文件 $RELEASE_DIR/logo.png"
    echo "将使用默认图标"
    ICON_PATH="applications-science"
else
    ICON_PATH="$RELEASE_DIR/logo.png"
    echo "✅ 找到图标文件: $ICON_PATH"
fi

# 确保启动脚本有执行权限
chmod +x "$RELEASE_DIR/run_drone_system.sh"
chmod +x "$RELEASE_DIR/drone_search_system"
echo "✅ 设置执行权限"

# 创建应用程序目录（如果不存在）
APPLICATIONS_DIR="$HOME/.local/share/applications"
if [ ! -d "$APPLICATIONS_DIR" ]; then
    mkdir -p "$APPLICATIONS_DIR"
    echo "✅ 创建应用程序目录: $APPLICATIONS_DIR"
fi

# 创建桌面图标文件
DESKTOP_FILE="$APPLICATIONS_DIR/drone_search_system.desktop"

cat > "$DESKTOP_FILE" << EOF
[Desktop Entry]
Name=无人机自主搜索系统
Name[en]=Drone Search System
Comment=无人机自主搜索和救援系统
Comment[en]=Autonomous Drone Search and Rescue System
Icon=$ICON_PATH
Exec=$RELEASE_DIR/run_drone_system.sh
Type=Application
Terminal=true
Categories=Science;Engineering;Robotics;
StartupNotify=true
Path=$RELEASE_DIR/
Keywords=drone;search;rescue;ros;rviz;
EOF

# 设置桌面文件权限
chmod +x "$DESKTOP_FILE"
echo "✅ 创建桌面图标文件: $DESKTOP_FILE"

# 创建桌面快捷方式（可选）
DESKTOP_DIR="$HOME/Desktop"
if [ -d "$DESKTOP_DIR" ]; then
    DESKTOP_SHORTCUT="$DESKTOP_DIR/drone_search_system.desktop"
    cp "$DESKTOP_FILE" "$DESKTOP_SHORTCUT"
    chmod +x "$DESKTOP_SHORTCUT"
    
    # 标记为可信任（Ubuntu 18.04+需要）
    if command -v gio &> /dev/null; then
        gio set "$DESKTOP_SHORTCUT" metadata::trusted true
        echo "✅ 创建桌面快捷方式: $DESKTOP_SHORTCUT"
    else
        echo "✅ 创建桌面快捷方式: $DESKTOP_SHORTCUT (可能需要手动信任)"
    fi
else
    echo "⚠️  桌面目录不存在，跳过桌面快捷方式创建"
fi

# 更新桌面数据库
if command -v update-desktop-database &> /dev/null; then
    update-desktop-database "$APPLICATIONS_DIR"
    echo "✅ 更新桌面数据库"
fi

# 刷新图标缓存
if command -v gtk-update-icon-cache &> /dev/null; then
    gtk-update-icon-cache -f -t "$HOME/.local/share/icons" 2>/dev/null || true
    echo "✅ 刷新图标缓存"
fi

echo ""
echo "🎉 桌面图标安装完成！"
echo ""
echo "现在你可以通过以下方式启动程序："
echo "1. 在应用程序菜单中搜索 '无人机自主搜索系统'"
echo "2. 双击桌面上的快捷方式（如果创建了）"
echo "3. 在文件管理器中双击 .desktop 文件"
echo ""
echo "📋 安装的文件："
echo "  - 应用程序图标: $DESKTOP_FILE"
if [ -f "$DESKTOP_DIR/drone_search_system.desktop" ]; then
    echo "  - 桌面快捷方式: $DESKTOP_DIR/drone_search_system.desktop"
fi
echo ""
echo "🔧 如果图标没有立即显示，请尝试："
echo "  - 注销并重新登录"
echo "  - 重启桌面环境"
echo "  - 手动刷新应用程序菜单"
echo ""
echo "✨ 享受你的无人机自主搜索系统！"
