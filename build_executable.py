#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
无人机自主搜索系统打包脚本
用于将start.py打包成可执行程序
"""

import os
import sys
import subprocess
import shutil
from pathlib import Path

def check_dependencies():
    """检查必要的依赖"""
    print("=== 检查依赖 ===")
    
    required_packages = [
        'pyinstaller',
        'numpy',
        'opencv-python',
        'psutil'
    ]
    
    missing_packages = []
    
    for package in required_packages:
        try:
            if package == 'pyinstaller':
                # 特殊处理PyInstaller检查
                pyinstaller_found = False

                # 方法1: 检查Python模块
                try:
                    __import__('PyInstaller')
                    pyinstaller_found = True
                except ImportError:
                    pass

                # 方法2: 检查用户本地路径
                if not pyinstaller_found:
                    local_pyinstaller = os.path.expanduser("~/.local/bin/pyinstaller")
                    if os.path.exists(local_pyinstaller):
                        pyinstaller_found = True

                # 方法3: 检查系统路径
                if not pyinstaller_found:
                    result = subprocess.run(['which', 'pyinstaller'],
                                          capture_output=True, text=True)
                    if result.returncode == 0:
                        pyinstaller_found = True

                if not pyinstaller_found:
                    raise ImportError()
            else:
                __import__(package.replace('-', '_'))
            print(f"✅ {package} 已安装")
        except ImportError:
            missing_packages.append(package)
            print(f"❌ {package} 未安装")
    
    if missing_packages:
        print(f"\n需要安装以下包: {', '.join(missing_packages)}")
        install_cmd = f"pip install {' '.join(missing_packages)}"
        print(f"运行命令: {install_cmd}")
        
        response = input("是否自动安装? (y/n): ")
        if response.lower() == 'y':
            subprocess.run(install_cmd, shell=True)
        else:
            print("请手动安装依赖后重新运行此脚本")
            return False
    
    return True

def prepare_build_environment():
    """准备构建环境"""
    print("\n=== 准备构建环境 ===")
    
    # 创建构建目录
    build_dir = Path("build_executable")
    if build_dir.exists():
        shutil.rmtree(build_dir)
    build_dir.mkdir()
    
    # 复制必要文件
    files_to_copy = [
        "start.py",
        "dashboard.py",
        "topics_subscriber.py",
        "topic_logger.py",
        "images_rc.py",
        "ball_pose_tracker.py",
        "topics_config.json",
        "my_config.rviz",
        "logo.png"
    ]
    
    for file in files_to_copy:
        if os.path.exists(file):
            shutil.copy2(file, build_dir)
            print(f"✅ 复制文件: {file}")
        else:
            print(f"⚠️  文件不存在: {file}")
    
    # 复制资源目录
    if os.path.exists("resource"):
        shutil.copytree("resource", build_dir / "resource")
        print("✅ 复制资源目录: resource")
    
    # 复制截图目录（如果存在）
    if os.path.exists("ball_screenshots"):
        shutil.copytree("ball_screenshots", build_dir / "ball_screenshots")
        print("✅ 复制截图目录: ball_screenshots")
    
    return build_dir

def create_spec_file(build_dir):
    """创建PyInstaller spec文件"""
    print("\n=== 创建spec文件 ===")
    
    spec_content = '''# -*- mode: python ; coding: utf-8 -*-

import os
import sys
from pathlib import Path

# 添加ROS路径
ros_paths = []
if 'ROS_PACKAGE_PATH' in os.environ:
    ros_paths = os.environ['ROS_PACKAGE_PATH'].split(':')

# 添加Python路径
python_paths = []
if 'PYTHONPATH' in os.environ:
    python_paths = os.environ['PYTHONPATH'].split(':')

# 合并所有路径
all_paths = ros_paths + python_paths + ['/opt/ros/noetic/lib/python3/dist-packages']

block_cipher = None

a = Analysis(
    ['start.py'],
    pathex=all_paths,
    binaries=[],
    datas=[
        ('resource', 'resource'),
        ('topics_config.json', '.'),
        ('my_config.rviz', '.'),
        ('logo.png', '.'),
        ('ball_screenshots', 'ball_screenshots') if os.path.exists('ball_screenshots') else None,
    ],
    hiddenimports=[
        'cv2',
        'numpy',
        'psutil',
        'rospy',
        'roslib',
        'geometry_msgs.msg',
        'sensor_msgs.msg',
        'visualization_msgs.msg',
        'std_msgs.msg',
        'rviz',
        'python_qt_binding.QtCore',
        'python_qt_binding.QtGui', 
        'python_qt_binding.QtWidgets',
        'PyQt5.QtCore',
        'PyQt5.QtGui',
        'PyQt5.QtWidgets',
        'dashboard',
        'topics_subscriber',
        'topic_logger',
        'images_rc'
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        'PySide2',
        'PySide2.QtCore',
        'PySide2.QtWidgets',
        'PySide2.QtGui',
        'tkinter',
        'matplotlib',
        'IPython',
        'jupyter',
    ],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

# 过滤掉None值
a.datas = [item for item in a.datas if item is not None]

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name='drone_search_system',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon='logo.png' if os.path.exists('logo.png') else None,
)
'''
    
    spec_file = build_dir / "drone_search_system.spec"
    with open(spec_file, 'w', encoding='utf-8') as f:
        f.write(spec_content)
    
    print(f"✅ 创建spec文件: {spec_file}")
    return spec_file

def create_startup_script(build_dir):
    """创建启动脚本"""
    print("\n=== 创建启动脚本 ===")
    
    startup_script = '''#!/bin/bash

# 无人机自主搜索系统启动脚本

echo "=== 无人机自主搜索系统 ==="
echo "正在启动系统..."

# 设置ROS环境
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
    echo "✅ ROS环境已加载"
else
    echo "❌ 未找到ROS环境，请确保ROS已正确安装"
    exit 1
fi

# 检查roscore
if ! pgrep -x "roscore" > /dev/null; then
    echo "启动roscore..."
    roscore &
    sleep 3
fi

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# 切换到程序目录（重要：确保工作目录正确）
cd "$SCRIPT_DIR"

# 显示当前工作目录
echo "当前工作目录: $(pwd)"

# 检查必要的目录
for dir in "ball_screenshots" "screenshots" "log"; do
    if [ ! -d "$dir" ]; then
        mkdir -p "$dir"
        echo "✅ 创建目录: $dir"
    fi
done

# 启动主程序
echo "启动主程序..."
./drone_search_system

echo "程序已退出"
'''
    
    script_file = build_dir / "start_drone_system.sh"
    with open(script_file, 'w', encoding='utf-8') as f:
        f.write(startup_script)
    
    # 设置执行权限
    os.chmod(script_file, 0o755)
    
    print(f"✅ 创建启动脚本: {script_file}")
    return script_file

def build_executable(build_dir, spec_file):
    """构建可执行文件"""
    print("\n=== 构建可执行文件 ===")

    # 查找PyInstaller的路径
    pyinstaller_paths = [
        os.path.expanduser("~/.local/bin/pyinstaller"),
        "/usr/local/bin/pyinstaller",
        "/usr/bin/pyinstaller"
    ]

    pyinstaller_cmd = None
    for path in pyinstaller_paths:
        if os.path.exists(path):
            pyinstaller_cmd = path
            break

    if not pyinstaller_cmd:
        # 尝试使用python -m pyinstaller
        pyinstaller_cmd = "python3 -m PyInstaller"

    print(f"使用PyInstaller: {pyinstaller_cmd}")

    # 切换到构建目录
    original_dir = os.getcwd()
    os.chdir(build_dir)

    try:
        # 运行PyInstaller
        cmd = f"{pyinstaller_cmd} --clean {spec_file.name}"
        print(f"运行命令: {cmd}")

        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        
        if result.returncode == 0:
            print("✅ 构建成功!")
            
            # 检查生成的文件
            exe_file = Path("dist/drone_search_system")
            if exe_file.exists():
                print(f"✅ 可执行文件已生成: {exe_file}")
                
                # 设置执行权限
                os.chmod(exe_file, 0o755)
                
                return exe_file
            else:
                print("❌ 未找到生成的可执行文件")
                return None
        else:
            print("❌ 构建失败!")
            print("错误输出:")
            print(result.stderr)
            return None
            
    finally:
        os.chdir(original_dir)

def create_package(build_dir, exe_file):
    """创建最终的发布包"""
    print("\n=== 创建发布包 ===")

    package_dir = Path("drone_search_system_release")  # 修改包名以匹配安装脚本
    if package_dir.exists():
        shutil.rmtree(package_dir)
    package_dir.mkdir()

    # 复制可执行文件
    shutil.copy2(build_dir / exe_file, package_dir)

    # 复制启动脚本并重命名
    startup_script = build_dir / "start_drone_system.sh"
    if startup_script.exists():
        # 重命名为与安装脚本期望的名称一致
        shutil.copy2(startup_script, package_dir / "run_drone_system.sh")

    # 复制必要的配置文件和脚本
    config_files = ["my_config.rviz", "topics_config.json", "logo.png", "install_desktop_icon.sh"]
    for config_file in config_files:
        src_file = build_dir / config_file
        if src_file.exists():
            shutil.copy2(src_file, package_dir)
            # 确保脚本文件有执行权限
            if config_file.endswith('.sh'):
                os.chmod(package_dir / config_file, 0o755)

    # 复制资源目录
    resource_src = build_dir / "resource"
    if resource_src.exists():
        shutil.copytree(resource_src, package_dir / "resource")

    # 创建必要的目录结构（即使为空）
    (package_dir / "ball_screenshots").mkdir(exist_ok=True)
    (package_dir / "screenshots").mkdir(exist_ok=True)
    (package_dir / "log").mkdir(exist_ok=True)

    # 复制现有的截图目录（如果存在）
    screenshots_src = build_dir / "ball_screenshots"
    if screenshots_src.exists():
        shutil.copytree(screenshots_src, package_dir / "ball_screenshots", dirs_exist_ok=True)
    
    # 创建README
    readme_content = '''# 无人机自主搜索系统

## 系统要求
- Ubuntu 20.04
- ROS Noetic
- Python 3.8+

## 安装说明
1. 确保ROS环境已正确安装和配置
2. 解压此包到任意目录
3. 运行安装脚本: ./install_desktop_icon.sh（可选）
4. 运行启动脚本: ./run_drone_system.sh

## 文件说明
- drone_search_system: 主程序可执行文件
- run_drone_system.sh: 启动脚本
- my_config.rviz: RViz配置文件
- topics_config.json: 话题配置文件
- resource/: 资源文件目录
- ball_screenshots/: 小球截图保存目录
- screenshots/: 一般截图保存目录
- log/: 日志文件目录

## 使用方法
1. 运行 ./run_drone_system.sh 启动系统
2. 系统会自动启动roscore（如果未运行）
3. 使用GUI界面控制无人机系统

## 路径说明
- 程序会优先在当前目录创建数据文件夹
- 如果当前目录不可写，会在用户主目录下创建 ~/drone_search_system/ 文件夹
- 截图和日志文件会保存在相应的数据目录中

## 注意事项
- 首次运行可能需要设置串口权限
- 确保所有ROS包已正确安装
- 如遇问题，请检查ROS环境变量设置
- 确保程序目录有写入权限，或者程序会自动使用用户目录
'''
    
    readme_file = package_dir / "README.md"
    with open(readme_file, 'w', encoding='utf-8') as f:
        f.write(readme_content)
    
    print(f"✅ 发布包已创建: {package_dir}")
    return package_dir

def main():
    """主函数"""
    print("无人机自主搜索系统打包工具")
    print("=" * 50)
    
    # 检查依赖
    if not check_dependencies():
        return False
    
    # 准备构建环境
    build_dir = prepare_build_environment()
    
    # 创建spec文件
    spec_file = create_spec_file(build_dir)
    
    # 创建启动脚本
    create_startup_script(build_dir)
    
    # 构建可执行文件
    exe_file = build_executable(build_dir, spec_file)
    
    if exe_file:
        # 创建发布包
        package_dir = create_package(build_dir, exe_file)
        
        print("\n" + "=" * 50)
        print("✅ 打包完成!")
        print(f"发布包位置: {package_dir}")
        print("运行方式:")
        print(f"  cd {package_dir}")
        print("  ./start_drone_system.sh")
        print("=" * 50)
        
        return True
    else:
        print("\n❌ 打包失败!")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
