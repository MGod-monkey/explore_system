#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
工具函数和全局常量模块

提供路径工具函数、全局样式和进程模式常量
"""

import os
import sys
import json

# =============================================================================
# 路径工具函数
# =============================================================================

def get_application_directory():
    """
    获取应用程序目录，兼容打包和非打包环境
    在打包环境下，返回可执行文件所在目录
    在开发环境下，返回脚本文件所在目录
    """
    if getattr(sys, 'frozen', False):
        # 打包环境：使用可执行文件所在目录
        application_path = os.path.dirname(sys.executable)
    else:
        # 开发环境：使用脚本文件所在目录
        application_path = os.path.dirname(os.path.abspath(__file__))

    return application_path

def get_data_directory(subdir_name):
    """
    获取数据目录（截图、日志等），确保在用户可写的位置
    优先使用程序目录，如果不可写则使用用户主目录
    """
    app_dir = get_application_directory()
    data_dir = os.path.join(app_dir, subdir_name)

    # 检查程序目录是否可写
    try:
        # 尝试在程序目录创建测试文件
        test_file = os.path.join(app_dir, '.write_test')
        with open(test_file, 'w') as f:
            f.write('test')
        os.remove(test_file)

        # 如果可写，使用程序目录
        if not os.path.exists(data_dir):
            os.makedirs(data_dir)
        return data_dir

    except (OSError, PermissionError):
        # 如果程序目录不可写，使用用户主目录
        user_data_dir = os.path.expanduser(f"~/drone_search_system/{subdir_name}")
        if not os.path.exists(user_data_dir):
            os.makedirs(user_data_dir)
        print(f"程序目录不可写，使用用户目录: {user_data_dir}")
        return user_data_dir

def get_config_file_path(filename):
    """
    获取配置文件路径，优先使用程序目录，如果不存在则使用用户目录
    """
    app_dir = get_application_directory()
    config_path = os.path.join(app_dir, filename)

    if os.path.exists(config_path):
        return config_path

    # 如果程序目录没有配置文件，检查用户目录
    user_config_path = os.path.expanduser(f"~/drone_search_system/{filename}")
    if os.path.exists(user_config_path):
        return user_config_path

    # 如果都不存在，返回程序目录路径（用于创建新文件）
    return config_path

# =============================================================================
# 进程配置加载
# =============================================================================

def load_processes_config():
    """
    从 JSON 配置文件加载进程配置
    返回配置字典，包含 catkin_workspace, log_directory, processes 列表
    """
    config_path = get_config_file_path("processes_config.json")
    
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config = json.load(f)
        
        # 按 order 排序进程列表
        if 'processes' in config:
            config['processes'] = sorted(config['processes'], key=lambda x: x.get('order', 999))
        
        print(f"已加载进程配置: {config_path}")
        return config
    except FileNotFoundError:
        print(f"警告: 进程配置文件不存在: {config_path}")
        return get_default_processes_config()
    except json.JSONDecodeError as e:
        print(f"警告: 进程配置文件解析错误: {e}")
        return get_default_processes_config()

def get_default_processes_config():
    """返回默认的进程配置（当配置文件不存在时使用）"""
    return {
        "catkin_workspace": "~/catkin_ws_dyn",
        "log_directory": "log",
        "processes": [
            {
                "name": "px4ctrl",
                "display_name": "PX4 飞控控制器",
                "start_command": "roslaunch px4ctrl run_node.launch",
                "stop_pattern": "roslaunch px4ctrl run_node.launch",
                "wait_seconds": 5,
                "order": 1
            },
            {
                "name": "rqt_reconfigure",
                "display_name": "ROS 参数配置工具",
                "start_command": "rosrun rqt_reconfigure rqt_reconfigure",
                "stop_pattern": "rqt_reconfigure",
                "wait_seconds": 5,
                "order": 2
            },
            {
                "name": "dynamic_predictor",
                "display_name": "动态障碍物预测器",
                "start_command": "roslaunch dynamic_predictor predictor_with_fake_detector.launch",
                "stop_pattern": "roslaunch dynamic_predictor predictor_with_fake_detector.launch",
                "wait_seconds": 10,
                "order": 3
            },
            {
                "name": "ego_planner",
                "display_name": "EGO 路径规划器",
                "start_command": "roslaunch ego_planner single_run_in_gazebo.launch",
                "stop_pattern": "roslaunch ego_planner single_run_in_gazebo.launch",
                "wait_seconds": 2,
                "order": 4
            }
        ]
    }

def get_process_stop_patterns():
    """获取用于停止进程的模式列表（兼容旧代码）"""
    config = load_processes_config()
    return [p.get('stop_pattern', p.get('start_command', '')) for p in config.get('processes', [])]

# =============================================================================
# 全局常量
# =============================================================================

# 进程模式 - 用于停止无人机系统（兼容旧代码，建议使用 get_process_stop_patterns()）
PROCESS_PATTERNS = get_process_stop_patterns()


# 全局样式常量
GLOBAL_STYLES = {
    'main_window': """
        QWidget {
            background-color: #1E2330;
            color: #FFFFFF;
        }
        QMainWindow::title {
            height: 35px;
        }
        QToolBar {
            background-color: #1A202C;
            border: none;
            spacing: 10px;
            padding: 5px;
        }
        QStatusBar {
            background-color: #1A202C;
            color: #FFFFFF;
        }
    """,
    'button_primary': """
        QPushButton {{
            background-color: #2C3E50;
            color: #FFFFFF;
            border: none;
            border-radius: 4px;
            padding: 6px 12px;
            font-weight: bold;
            min-width: {min_width}px;
            min-height: {min_height}px;
        }}
        QPushButton:hover {{
            background-color: #3498DB;
        }}
        QPushButton:pressed {{
            background-color: #2980B9;
        }}
    """,
    'groupbox': """
        QGroupBox {
            color: #3498DB;
            font-weight: bold;
            border: 1px solid #3498DB;
            border-radius: 5px;
            padding: 10px;
            margin-top: 10px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            subcontrol-position: top center;
            padding: 0 5px;
        }
    """,
    'label': """
        QLabel {
            font-size: 12pt;
            font-weight: bold;
            padding: 5px;
        }
    """
}
