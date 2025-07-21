#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 标准库导入
import sys
import time
import os
import json
import math
import subprocess
import threading
from collections import defaultdict

# ROS相关导入
import roslib; roslib.load_manifest('rviz_python_tutorial')
import rospy
from geometry_msgs.msg import PoseStamped

# 第三方库导入
import numpy as np
import cv2

# 进程管理库
try:
    import psutil
except ImportError:
    print("警告: 未能导入psutil库，进程管理功能将受限")
    try:
        subprocess.call([sys.executable, "-m", "pip", "install", "psutil"])
        import psutil
        print("已成功安装psutil库")
    except:
        print("自动安装psutil失败，请手动安装: pip install psutil")

# 自定义模块导入
try:
    from topics_subscriber import TopicsSubscriber
except ImportError:
    print("无法导入topics_subscriber模块")
    TopicsSubscriber = None

try:
    from dashboard import UIButton
except ImportError:
    print("无法导入dashboard模块")
    UIButton = None

try:
    from topic_logger import TopicLogger
except ImportError:
    print("无法导入topic_logger模块")
    TopicLogger = None

# Qt相关导入
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

# 尝试导入QTextCodec，如果失败则忽略
try:
    from python_qt_binding.QtCore import QTextCodec
except ImportError:
    QTextCodec = None
    print("警告: QTextCodec不可用，跳过编码设置")

from python_qt_binding.QtCore import QTimer, QPropertyAnimation, QEasingCurve
try:
    from python_qt_binding.QtCore import pyqtSignal
except ImportError:
    try:
        from PyQt5.QtCore import pyqtSignal
    except ImportError:
        try:
            from PyQt4.QtCore import pyqtSignal
        except ImportError:
            print("警告: 无法导入pyqtSignal，某些功能可能不可用")
            pyqtSignal = None

# 资源文件导入
try:
    import images_rc
except ImportError:
    print("警告: 无法导入images_rc资源文件，请确保已使用pyrcc5编译资源文件")

# RViz导入
from rviz import bindings as rviz

# 路径工具函数
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

# 全局常量
PROCESS_PATTERNS = [
    "sh shfiles/run.sh",
    "roslaunch mavros px4.launch",
    "roslaunch vins fast_drone_250.launch",
    "roslaunch onboard_detector run_detector.launch",
    "roslaunch px4ctrl run_ctrl.launch",
    "roslaunch ego_planner single_run_in_exp.launch",
    "roslaunch realsense2_camera rs_camera.launch",
    "vins_to_mavros_node",
    "pose_to_odom_converter_node",
    "roslaunch exploration_manager exploration.launch",
    "roslaunch yolo_detector yolo_ros.launch",
    "roslaunch sort_ros sort_ros.launch",
    "python3 ball_pose_tracker.py",
    "rosrun exploration_manager fuel_nav"
]

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

class MyViz(QMainWindow):
    """无人机自主搜索系统主窗口类"""

    # 定义信号，用于线程安全的UI更新（如果pyqtSignal可用）
    if pyqtSignal is not None:
        image_update_signal = pyqtSignal()
        bird_view_update_signal = pyqtSignal()

    def __init__(self):
        super(MyViz, self).__init__()

        # 初始化基本属性
        self._init_basic_attributes()

        # 初始化UI
        self._init_ui()

        # 初始化RViz
        self._init_rviz()

        # 创建布局
        self._create_layouts()

        # 初始化定时器（合并多个定时器）
        self._init_timers()

        # 连接信号到槽函数（如果信号可用）
        if pyqtSignal is not None and hasattr(self, 'image_update_signal'):
            self.image_update_signal.connect(self.updateImageDisplay)
            self.bird_view_update_signal.connect(self.updateBirdViewDisplay)

        # 延迟初始化话题订阅器
        QTimer.singleShot(2000, self.setupTopicSubscriber)

    def _init_basic_attributes(self):
        """初始化基本属性"""
        # 获取屏幕信息
        self.desktop = QDesktopWidget()
        self.screen_geometry = self.desktop.availableGeometry(self.desktop.primaryScreen())
        self.screen_width = self.screen_geometry.width()
        self.screen_height = self.screen_geometry.height()
        print(f"检测到屏幕分辨率: {self.screen_width}x{self.screen_height}")

        # 计算自适应尺寸
        self.calculateAdaptiveSizes()

        # 数据变量
        self.battery_percentage = 100.0
        self.battery_voltage = 12.0
        self.camera_image = None
        self.depth_image = None
        self.bird_view_image = None
        self.pitch = 0
        self.roll = 0
        self.speed = 0
        self.linear_speed = 0
        self.angular_speed = 0

        # 状态变量
        self.detected_markers = set()
        self.topics_with_data = defaultdict(bool)
        self.current_image_mode = "rgb"
        self.ball_screenshots = {}

        # UI状态变量
        self.sidebar_expanded = True
        self.right_sidebar_expanded = True
        self.left_sidebar_pinned = False
        self.right_sidebar_pinned = False
        self.enable_sidebar_hover = False

        # 进程管理
        self.processes = {}
        self.log_files = {}

        # 帧率计算
        self.frame_count = 0
        self.last_frame_time = time.time()

        # 话题订阅器
        self.topic_subscriber = None
        self.log_window = None

        # 截图目录 - 使用新的路径工具函数
        self.screenshots_dir = get_data_directory("screenshots")

    def _init_ui(self):
        """初始化UI设置"""
        # 设置字体
        font = QFont("WenQuanYi Micro Hei", 10)
        QApplication.setFont(font)

        # 设置图标和标题
        self.setWindowIcon(QIcon("logo.png"))
        self.setWindowTitle("无人机自主搜索系统")

        # 创建中央控件
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        # 设置全局样式
        self.setStyleSheet(GLOBAL_STYLES['main_window'])

        # 设置窗口最小尺寸
        min_width = self.adaptive_left_width + self.adaptive_right_width + 500
        min_height = 600
        self.setMinimumSize(min_width, min_height)

        # 绑定事件
        self.resizeEvent = self.onResize
    def _init_rviz(self):
        """初始化RViz组件"""
        # 设置RViz显示
        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath("")
        self.frame.initialize()

        # 读取配置文件
        try:
            reader = rviz.YamlConfigReader()
            config = rviz.Config()
            reader.readFile(config, "my_config.rviz")
            self.frame.load(config)
        except Exception as e:
            print(f"加载RViz配置文件失败: {e}")

        # 禁用菜单栏、状态栏和"隐藏停靠"按钮
        self.frame.setMenuBar(None)
        self.frame.setStatusBar(None)
        self.frame.setHideButtonVisibility(True)

        # 获取VisualizationManager实例
        self.manager = self.frame.getManager()
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt(0)

    def _init_timers(self):
        """初始化定时器（合并多个定时器以减少资源占用）"""
        # 主更新定时器 - 合并多个高频更新
        self.main_update_timer = QTimer(self)
        self.main_update_timer.timeout.connect(self._main_update_cycle)
        self.main_update_timer.start(100)  # 10Hz，平衡性能和响应性

        # 图像更新检查定时器 - 确保图像能够及时更新
        self.image_update_timer = QTimer(self)
        self.image_update_timer.timeout.connect(self._check_image_update)
        self.image_update_timer.start(50)  # 20Hz，确保图像更新及时

        # 鼠标跟踪定时器
        self.sidebar_hover_timer = QTimer(self)
        self.sidebar_hover_timer.timeout.connect(self.checkMousePosition)
        self.sidebar_hover_timer.start(100)  # 降低频率到100ms

        # 延迟初始化定时器
        QTimer.singleShot(1000, self.setupAllOverlaysAndHideSidebar)
        QTimer.singleShot(1000, self.updateImageSizes)

    def _main_update_cycle(self):
        """主更新循环 - 合并多个更新操作"""
        try:
            # 更新状态栏
            self.updateStatusBar()

            # 移除图像显示的频率限制，让图像能够实时更新
            # 图像更新现在主要由话题回调函数触发，这里只作为备用更新机制
            # 注释掉原来的频率限制逻辑
            # if self.frame_count % 3 == 0:  # 每300ms更新一次图像
            #     self.updateImageDisplay()
            #     self.updateBirdViewDisplay()

            # 更新悬浮组件数据（降低频率）
            if self.frame_count % 5 == 0:  # 每500ms更新一次悬浮数据
                # RViz悬浮面板数据现在直接在话题回调中更新，无需在此处调用
                # self.updateOverlayData()
                # 更新指南针和姿态组件数据
                self._update_compass_data()
                self._update_attitude_widget_data()

            # 更新悬浮组件位置（降低频率）
            if self.frame_count % 2 == 0:  # 每200ms更新一次位置
                # 检查RViz框架是否存在且有效
                if hasattr(self, 'frame') and self.frame and self.frame.isVisible():
                    if hasattr(self, 'rviz_overlay') and hasattr(self, 'updateOverlayPosition'):
                        self.updateOverlayPosition()
                    if hasattr(self, 'compass') and hasattr(self, 'updateCompassPosition'):
                        self.updateCompassPosition()
                    if hasattr(self, 'attitude_widget') and hasattr(self, 'updateAttitudeWidgetPosition'):
                        self.updateAttitudeWidgetPosition()

            self.frame_count += 1

        except Exception as e:
            print(f"主更新循环错误: {e}")

    def _check_image_update(self):
        """检查并更新图像显示 - 确保图像能够及时更新"""
        try:
            # 检查是否有图像标签
            if not hasattr(self, 'image_label') or not self.image_label:
                return

            # 根据当前模式检查是否有新的图像数据需要显示
            should_update = False

            if self.current_image_mode == "rgb" and self.camera_image is not None:
                # 检查RGB图像是否需要更新
                should_update = True
            elif self.current_image_mode == "depth" and self.depth_image is not None:
                # 检查深度图像是否需要更新
                should_update = True

            # 如果需要更新且当前显示的是占位符文本，则强制更新
            if should_update:
                current_pixmap = self.image_label.pixmap()
                if current_pixmap is None or current_pixmap.isNull():
                    # 当前没有显示图像，强制更新
                    self.updateImageDisplay()

        except Exception as e:
            # 静默处理错误，避免干扰主程序
            pass

    def _update_compass_data(self):
        """更新指南针数据"""
        try:
            if hasattr(self, 'topic_subscriber') and self.topic_subscriber and hasattr(self, 'compass') and self.compass:
                # 检查姿态话题是否活跃
                if self.topic_subscriber.is_topic_active("attitude"):
                    # 尝试从姿态数据获取航向信息
                    attitude_data = self.topic_subscriber.get_latest_data("attitude")
                    if attitude_data and "yaw" in attitude_data:
                        # 获取原始yaw值
                        yaw_value = attitude_data.get("yaw", 0)
                        if isinstance(yaw_value, list):
                            if len(yaw_value) > 0:
                                yaw_value = yaw_value[0]  # 取列表的第一个元素
                            else:
                                yaw_value = 0  # 空列表时使用默认值

                        # 限制在360度范围内
                        if yaw_value > 360:
                            yaw_value = yaw_value % 360
                        elif yaw_value < -360:
                            yaw_value = yaw_value % 360

                        # 直接使用yaw值作为指南针角度
                        self.compass.set_heading(-yaw_value)

        except Exception as e:
            print(f"更新指南针数据时出错: {str(e)}")

    def _update_attitude_widget_data(self):
        """更新姿态指示器数据"""
        try:
            if hasattr(self, 'topic_subscriber') and self.topic_subscriber and hasattr(self, 'attitude_widget') and self.attitude_widget:
                # 检查姿态话题是否活跃
                if self.topic_subscriber.is_topic_active("attitude"):
                    # 从姿态数据获取俯仰和滚转角度
                    attitude_data = self.topic_subscriber.get_latest_data("attitude")
                    if attitude_data:
                        # 获取俯仰角并检查是否为列表
                        pitch_value = attitude_data.get("pitch", 0)
                        if isinstance(pitch_value, list):
                            if len(pitch_value) > 0:
                                pitch_value = pitch_value[0]
                            else:
                                pitch_value = 0

                        # 获取滚转角并检查是否为列表
                        roll_value = attitude_data.get("roll", 0)
                        if isinstance(roll_value, list):
                            if len(roll_value) > 0:
                                roll_value = roll_value[0]
                            else:
                                roll_value = 0

                        # 更新姿态指示器
                        self.attitude_widget.update_attitude(pitch_value, roll_value)

        except Exception as e:
            print(f"更新姿态指示器数据时出错: {str(e)}")

    def _update_overlay_positions(self):
        """立即更新所有悬浮窗口位置"""
        try:
            # 确保RViz框架已经完成布局更新
            if hasattr(self, 'frame') and self.frame:
                # 强制更新RViz框架的几何信息
                self.frame.update()
                QApplication.processEvents()

            # 更新各个悬浮窗口位置
            if hasattr(self, 'updateOverlayPosition'):
                self.updateOverlayPosition()
            if hasattr(self, 'updateCompassPosition'):
                self.updateCompassPosition()
            if hasattr(self, 'updateAttitudeWidgetPosition'):
                self.updateAttitudeWidgetPosition()
        except Exception as e:
            print(f"更新悬浮窗口位置时出错: {e}")

    def _create_styled_button(self, text, style_type="primary", min_width=120, min_height=30):
        """创建带样式的按钮，减少重复代码"""
        button = QPushButton(text)

        if style_type == "primary":
            style = GLOBAL_STYLES['button_primary'].format(
                min_width=min_width,
                min_height=min_height
            )
        else:
            # 可以扩展其他样式类型
            style = GLOBAL_STYLES['button_primary'].format(
                min_width=min_width,
                min_height=min_height
            )

        button.setStyleSheet(style)
        return button

    def _safe_execute(self, func, error_msg="操作执行失败", *args, **kwargs):
        """安全执行函数，统一错误处理"""
        try:
            return func(*args, **kwargs)
        except Exception as e:
            print(f"{error_msg}: {str(e)}")
            return None

    def _update_label_safely(self, label, text, default_text="数据获取失败"):
        """安全更新标签文本"""
        try:
            if hasattr(self, label) and getattr(self, label):
                getattr(self, label).setText(text)
        except Exception as e:
            print(f"更新标签 {label} 失败: {e}")
            if hasattr(self, label) and getattr(self, label):
                getattr(self, label).setText(default_text)

    def _scale_and_set_pixmap(self, label_name, pixmap, width=None, height=None):
        """统一的图像缩放和设置函数"""
        if not hasattr(self, label_name) or not getattr(self, label_name):
            return False

        try:
            label = getattr(self, label_name)
            if width is None:
                width = self.adaptive_image_width
            if height is None:
                height = self.adaptive_image_height

            scaled_pixmap = pixmap.scaled(
                width, height,
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            label.setPixmap(scaled_pixmap)
            return True
        except Exception as e:
            print(f"设置图像到 {label_name} 失败: {e}")
            return False

    def _cleanup_resources(self):
        """清理资源，减少内存泄漏"""
        try:
            # 停止所有定时器
            if hasattr(self, 'main_update_timer'):
                self.main_update_timer.stop()
            if hasattr(self, 'image_update_timer'):
                self.image_update_timer.stop()
            if hasattr(self, 'sidebar_hover_timer'):
                self.sidebar_hover_timer.stop()

            # 清理图像数据
            self.camera_image = None
            self.depth_image = None
            self.bird_view_image = None

            # 清理话题订阅器
            if self.topic_subscriber:
                self.topic_subscriber = None

            # 清理动画对象
            if hasattr(self, 'sidebar_animation'):
                try:
                    self.sidebar_animation.finished.disconnect()
                    self.sidebar_animation.valueChanged.disconnect()
                    self.sidebar_animation.stop()
                    self.sidebar_animation.deleteLater()
                except:
                    pass

            if hasattr(self, 'right_sidebar_animation'):
                try:
                    self.right_sidebar_animation.finished.disconnect()
                    self.right_sidebar_animation.valueChanged.disconnect()
                    self.right_sidebar_animation.stop()
                    self.right_sidebar_animation.deleteLater()
                except:
                    pass

            print("资源清理完成")
        except Exception as e:
            print(f"资源清理时出错: {e}")

    def closeEvent(self, event):
        """窗口关闭事件处理 - 优化版本"""
        try:
            print("正在关闭应用程序...")

            # 清理资源
            self._cleanup_resources()

            # 关闭悬浮窗口
            if hasattr(self, 'overlay_widget') and self.overlay_widget:
                self.overlay_widget.close()
            if hasattr(self, 'compass') and self.compass:
                self.compass.close()
            if hasattr(self, 'attitude_widget') and self.attitude_widget:
                self.attitude_widget.close()
            if hasattr(self, 'log_window') and self.log_window:
                self.log_window.close()

            # 终止进程
            self._safe_execute(self.silentStopDroneSystem, "停止进程失败")

            # 接受关闭事件
            event.accept()

        except Exception as e:
            print(f"关闭事件处理失败: {e}")
            event.accept()  # 即使出错也要关闭

    def _create_layouts(self):
        """创建主要布局"""
        ## 创建主布局
        main_layout = QVBoxLayout(self.central_widget)
        main_layout.setContentsMargins(10, 15, 10, 10)  # 增加上边距
        main_layout.setSpacing(10)  # 增加组件间距

        ## 创建标题和工具栏区域
        header_widget = QWidget()
        header_widget.setStyleSheet("background-color: #1A202C; border-radius: 5px;")
        header_widget.setMaximumHeight(120)  # 提高标题栏最大高度
        header_layout = QVBoxLayout(header_widget)
        header_layout.setContentsMargins(5, 5, 5, 5)
        header_layout.setSpacing(2)
        
        # 创建标题标签
        title_label = QLabel("无人机自主搜索系统")
        title_label.setStyleSheet("font-size: 24pt; color: #3498DB; padding: 10px; font-weight: bold;")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setMinimumWidth(500)  # 设置最小宽度
        title_label.setMaximumHeight(60)  # 增加标题标签高度
        header_layout.addWidget(title_label)
        
        # 创建工具栏
        toolbar_widget = QWidget()
        toolbar_layout = QHBoxLayout(toolbar_widget)
        toolbar_layout.setContentsMargins(5, 0, 5, 0)
        toolbar_layout.setSpacing(10)
        
        # 左侧功能按钮
        function_layout = QHBoxLayout()
        function_layout.setSpacing(8)  # 减少按钮间距
        
        # # 启动程序按钮
        # start_button = QPushButton("启动程序")
        # start_button.setIcon(QIcon(":/images/icons/start.svg"))
        # start_button.setIconSize(QSize(24, 24))
        # start_button.setStyleSheet("background-color: #27AE60; text-align: center; padding-left: 5px;")
        # start_button.setMinimumWidth(120)
        # start_button.setMaximumHeight(36)
        # function_layout.addWidget(start_button)
        
        # # 操控无人机按钮
        # control_button = QPushButton("操控无人机")
        # control_button.setIcon(QIcon(":/images/icons/joystick.png"))
        # control_button.setIconSize(QSize(24, 24))
        # control_button.setMinimumWidth(120)
        # control_button.setMaximumHeight(36)
        # function_layout.addWidget(control_button)
        
        # # 设置按钮，用于显示/隐藏Display面板
        # self.settings_button = QPushButton("设置")
        # self.settings_button.setIcon(QIcon(":/images/setting.png"))
        # self.settings_button.setIconSize(QSize(24, 24))
        # self.settings_button.setMinimumWidth(120)
        # self.settings_button.setMaximumHeight(36)
        # self.settings_button.clicked.connect(self.toggleRVizDisplayPanel)
        # function_layout.addWidget(self.settings_button)
        
        toolbar_layout.addLayout(function_layout)
        toolbar_layout.addStretch(1)  # 添加弹性空间
        
        # 添加日志按钮
        self.log_button = QPushButton("日志显示")
        self.log_button.setIcon(QIcon(":/images/icons/log.svg"))
        self.log_button.setIconSize(QSize(24, 24))
        self.log_button.setStyleSheet("""
            QPushButton {
                background-color: #2C3E50;
                color: #FFFFFF;
                border: none;
                border-radius: 4px;
                padding: 6px 12px;
                font-weight: bold;
                min-width: 120px;
                min-height: 36px;
                text-align: center;
            }
            QPushButton:hover {
                background-color: #3498DB;
            }
            QPushButton:pressed {
                background-color: #2980B9;
            }
            QPushButton:checked {
                background-color: #2980B9;
                border: 1px solid #1ABC9C;
            }
        """)
        self.log_button.setCheckable(True)  # 可以切换选中状态
        self.log_button.clicked.connect(self.toggleLogWindow)
        function_layout.addWidget(self.log_button)
        
        # 添加间隔
        function_layout.addSpacing(15)
        
        # 右侧状态显示
        status_layout = QHBoxLayout()
        status_layout.setSpacing(8)  # 减少状态组件间距
        
        # 电池状态显示
        self.battery_icon_label = QLabel()
        self.battery_icon_label.setPixmap(QPixmap(":/images/icons/battery_100.svg").scaled(24, 24, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.battery_icon_label.setAlignment(Qt.AlignCenter)
        status_layout.addWidget(self.battery_icon_label)
        
        # 电压图标
        voltage_icon_label = QLabel()
        voltage_icon_label.setPixmap(QPixmap(":/images/icons/voltage.svg").scaled(24, 24, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        voltage_icon_label.setAlignment(Qt.AlignCenter)
        status_layout.addWidget(voltage_icon_label)
        
        # 电压数值显示
        self.voltage_label = QLabel("0.0 V")
        self.voltage_label.setStyleSheet("color: #3498DB; font-weight: bold;")
        self.voltage_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        status_layout.addWidget(self.voltage_label)
        
        # 添加右侧状态栏
        toolbar_layout.addLayout(status_layout)
        
        header_layout.addWidget(toolbar_widget)
        main_layout.addWidget(header_widget)
        
        # 创建中间的大型分割器，包含左侧边栏和RViz显示区域
        self.main_splitter = QSplitter(Qt.Horizontal)
        
        # 创建左侧边栏，用于显示速度表盘和其他信息
        self.left_sidebar = QWidget()
        self.left_sidebar.setFixedWidth(self.adaptive_left_width)  # 使用自适应宽度
        # 使用QSizePolicy允许垂直方向缩放
        self.left_sidebar.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)
        left_sidebar_layout = QVBoxLayout(self.left_sidebar)
        left_sidebar_layout.setContentsMargins(10, 10, 10, 10)  # 增加边距
        left_sidebar_layout.setSpacing(10)  # 减小组件间距
        
        # 添加无人机状态组件 - 现代化卡片设计
        status_group = QGroupBox("🚁 无人机状态")
        status_group.setStyleSheet("""
            QGroupBox {
                color: #3498DB;
                font-size: 16pt;
                font-weight: bold;
                border: 2px solid #3498DB;
                border-radius: 12px;
                padding: 15px;
                margin-top: 20px;
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 rgba(52, 152, 219, 0.1),
                    stop:1 rgba(26, 32, 44, 0.8));
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top center;
                padding: 0 10px;
                background-color: #1E2330;
                border-radius: 6px;
            }
        """)
        status_group_layout = QVBoxLayout(status_group)
        status_group_layout.setContentsMargins(0, 0, 0, 0)  # 进一步减少内边距
        status_group_layout.setSpacing(4)  # 进一步减少组件间距

        # 创建无人机状态信息容器，使用现代化卡片布局
        info_container = QWidget()
        info_container.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        info_container.setStyleSheet("""
            QWidget {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 rgba(44, 62, 80, 0.9),
                    stop:1 rgba(26, 32, 44, 0.9));
                border-radius: 10px;
                border: 1px solid rgba(52, 152, 219, 0.3);
            }
        """)
        info_layout = QVBoxLayout(info_container)
        info_layout.setContentsMargins(3, 3, 3, 3)  # 进一步减少内边距
        info_layout.setSpacing(3)  # 进一步减少组件间距
        
        # 创建状态卡片容器
        self.createStatusCards(info_layout)

        
        status_group_layout.addWidget(info_container, 1)  # 使用拉伸系数1
        
        # 添加状态组到左侧边栏，增加拉伸系数给状态组更多空间
        left_sidebar_layout.addWidget(status_group, 3)  # 增加拉伸系数，给状态组更多空间

        # 添加功能区域组件（与状态区分离）
        function_group = QGroupBox("🎮 控制中心")
        function_group.setStyleSheet("""
            QGroupBox {
                color: #3498DB;
                font-size: 16pt;
                font-weight: bold;
                border: 2px solid #3498DB;
                border-radius: 12px;
                padding: 15px;
                margin-top: 20px;
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 rgba(52, 152, 219, 0.1),
                    stop:1 rgba(26, 32, 44, 0.8));
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top center;
                padding: 0 10px;
                background-color: #1E2330;
                border-radius: 6px;
            }
        """)
        function_group.setObjectName("function_group")  # 设置对象名，方便后续查找
        self.function_group = function_group  # 保存引用
        function_group_layout = QVBoxLayout(function_group)
        function_group_layout.setContentsMargins(0, 0, 0, 0)  # 移除所有内边距
        function_group_layout.setSpacing(0)  # 移除间距
        
        # 添加功能按钮区域
        function_area = QFrame()
        function_area.setFrameShape(QFrame.StyledPanel)
        function_area.setStyleSheet("""
            QFrame {
                background-color: #1A202C; 
                border-radius: 10px; 
                border: 1px solid #3498DB;
            }
        """)
        # 移除高度限制，允许拉伸填充
        function_area.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # 创建垂直布局来放置功能按钮
        function_container = QVBoxLayout(function_area)
        function_container.setContentsMargins(5, 5, 5, 5)  # 减小内边距
        function_container.setSpacing(0)  # 减小组件间距
        
        # 创建扇形控制按钮组件
        if UIButton:
            self.ui_button = UIButton()
            # 根据屏幕尺寸调整控件大小，小屏幕使用更小的尺寸
            if self.screen_width <= 1366:  # 小屏幕
                min_size = 250
                max_size = 400
            elif self.screen_width <= 1920:  # 中等屏幕
                min_size = 300
                max_size = 500
            else:  # 大屏幕
                min_size = 350
                max_size = 600

            self.ui_button.setMinimumSize(min_size, min_size)
            self.ui_button.setMaximumSize(max_size, max_size)
            # 连接信号到对应的槽函数
            self.ui_button.centerClicked.connect(self.startDroneSystem)  # 中间按钮 - 一键启动
            # self.ui_button.topClicked.connect(self.onTopButtonClick)     # 顶部按钮 - 一键返航
            self.ui_button.leftClicked.connect(self.publishNavigationGoal)  # 左侧按钮 - 开始探索
            self.ui_button.rightClicked.connect(self.stopDroneSystem)    # 右侧按钮 - 停止程序
            # 底部按钮暂时不连接功能

            # 添加到功能区域，居中对齐
            function_container.addWidget(self.ui_button, 0, Qt.AlignCenter)
        else:
            # 如果UIButton不可用，使用原来的按钮布局
            # 使用GridLayout进行布局，方便按钮的定位
            function_grid = QWidget()
            function_layout = QGridLayout(function_grid)
            function_layout.setContentsMargins(5, 2, 5, 2)  # 减小内边距
            function_layout.setSpacing(3)  # 进一步减小组件间距
            
            # 创建上方按钮 - 一键返航
            return_home_btn = QPushButton("一键返航")
            return_home_btn.setCursor(Qt.PointingHandCursor)  # 设置鼠标悬停时的光标为手型
            return_home_btn.setStyleSheet("""
                QPushButton {
                    background-color: #8E44AD;
                    color: white;
                    border-radius: 8px;
                    font-size: 12pt;
                    font-weight: bold;
                    padding: 8px;
                    min-height: 40px;
                    border: none;
                }
                QPushButton:hover {
                    background-color: #9B59B6;
                }
                QPushButton:pressed {
                    background-color: #7D3C98;
                }
            """)
            # 添加阴影效果
            shadow = QGraphicsDropShadowEffect()
            shadow.setBlurRadius(10)
            shadow.setColor(QColor(0, 0, 0, 60))
            shadow.setOffset(2, 2)
            return_home_btn.setGraphicsEffect(shadow)
            return_home_btn.clicked.connect(self.onTopButtonClick)
            
            function_layout.addWidget(return_home_btn, 0, 1)
            
            # 创建左侧按钮 - 开始探索 - 文字竖向排列
            explore_btn = QPushButton()
            explore_btn.setCursor(Qt.PointingHandCursor)  # 设置鼠标悬停时的光标为手型
            explore_btn.setStyleSheet("""
                QPushButton {
                    background-color: #27AE60;
                    color: white;
                    border-radius: 8px;
                    font-size: 12pt;
                    font-weight: bold;
                    padding: 20px 5px;
                    min-width: 40px;
                    min-height: 100px;
                    border: none;
                }
                QPushButton:hover {
                    background-color: #2ECC71;
                }
                QPushButton:pressed {
                    background-color: #229954;
                }
            """)
            # 添加阴影效果
            shadow2 = QGraphicsDropShadowEffect()
            shadow2.setBlurRadius(10)
            shadow2.setColor(QColor(0, 0, 0, 60))
            shadow2.setOffset(2, 2)
            explore_btn.setGraphicsEffect(shadow2)
            # 创建竖向文字标签
            explore_label = QLabel("开\n始\n探\n索")
            explore_label.setAlignment(Qt.AlignCenter)
            explore_label.setStyleSheet("color: white; background-color: transparent; font-size: 12pt; font-weight: bold;")
            # 添加标签到按钮
            explore_layout = QVBoxLayout(explore_btn)
            explore_layout.setContentsMargins(5, 5, 5, 5)
            explore_layout.addWidget(explore_label, 0, Qt.AlignCenter)
            
            # 连接开始探索按钮的点击事件
            explore_btn.clicked.connect(self.publishNavigationGoal)
            
            function_layout.addWidget(explore_btn, 1, 0)
            
            # 创建中间按钮 - 一键启动（无背景色）
            start_btn = QPushButton()
            start_btn.setCursor(Qt.PointingHandCursor)  # 设置鼠标悬停时的光标为手型
            start_btn.setStyleSheet("""
                QPushButton {
                    background-color: transparent;
                    color: white;
                    border-radius: 50px;  /* 完全圆形 */
                    font-size: 14pt;
                    font-weight: bold;
                    min-width: 100px;
                    min-height: 100px;
                    max-width: 100px;
                    max-height: 100px;
                    border: none;  /* 无边框 */
                }
                QPushButton:hover {
                    background-color: rgba(39, 174, 96, 30);  /* 绿色半透明悬停效果 */
                }
                QPushButton:pressed {
                    background-color: rgba(39, 174, 96, 50);  /* 绿色半透明按下效果 */
                }
            """)
            # 添加阴影效果
            shadow3 = QGraphicsDropShadowEffect()
            shadow3.setBlurRadius(15)
            shadow3.setColor(QColor(39, 174, 96, 80))  # 使用绿色阴影
            shadow3.setOffset(0, 0)
            start_btn.setGraphicsEffect(shadow3)
            
            # 创建垂直布局来排列图标和文字
            start_layout = QVBoxLayout(start_btn)
            start_layout.setContentsMargins(5, 5, 5, 5)  # 减小内边距
            start_layout.setSpacing(2)  # 减小组件间距
            
            # 添加图标
            start_icon_label = QLabel()
            # 创建一个绿色滤镜，将图标颜色转换为绿色
            icon_pixmap = QPixmap(":/images/icons/start.svg").scaled(48, 48, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            # 创建一个绿色滤镜效果
            icon_painter = QPainter(icon_pixmap)
            icon_painter.setCompositionMode(QPainter.CompositionMode_SourceIn)
            icon_painter.fillRect(icon_pixmap.rect(), QColor("#27AE60"))  # 绿色
            icon_painter.end()
            
            start_icon_label.setPixmap(icon_pixmap)
            start_icon_label.setAlignment(Qt.AlignCenter)
            start_layout.addWidget(start_icon_label, 0, Qt.AlignCenter)
            
            # 添加文字标签
            start_text_label = QLabel("一键启动")
            start_text_label.setStyleSheet("color: #27AE60; background-color: transparent; font-size: 14pt; font-weight: bold; border: none;")
            start_text_label.setAlignment(Qt.AlignCenter)
            start_layout.addWidget(start_text_label, 0, Qt.AlignCenter)
            
            # 连接一键启动按钮的点击事件
            start_btn.clicked.connect(self.startDroneSystem)
            
            function_layout.addWidget(start_btn, 1, 1)
            
            # 创建右侧按钮 - 停止程序 - 文字竖向排列
            future_btn_right = QPushButton()
            future_btn_right.setCursor(Qt.PointingHandCursor)  # 设置鼠标悬停时的光标为手型
            future_btn_right.setStyleSheet("""
                QPushButton {
                    background-color: #E74C3C;  /* 红色背景 */
                    color: white;
                    border-radius: 8px;
                    font-size: 12pt;
                    font-weight: bold;
                    padding: 20px 5px;
                    min-width: 40px;
                    min-height: 100px;
                    border: none;
                }
                QPushButton:hover {
                    background-color: #C0392B;  /* 深红色悬停效果 */
                }
                QPushButton:pressed {
                    background-color: #A93226;  /* 更深红色按下效果 */
                }
            """)
            # 添加阴影效果
            shadow4 = QGraphicsDropShadowEffect()
            shadow4.setBlurRadius(10)
            shadow4.setColor(QColor(0, 0, 0, 60))
            shadow4.setOffset(2, 2)
            future_btn_right.setGraphicsEffect(shadow4)
            # 创建竖向文字标签
            future_label = QLabel("停\n止\n程\n序")
            future_label.setAlignment(Qt.AlignCenter)
            future_label.setStyleSheet("color: white; background-color: transparent; font-size: 12pt; font-weight: bold;")
            # 添加标签到按钮
            future_layout = QVBoxLayout(future_btn_right)
            future_layout.setContentsMargins(5, 5, 5, 5)
            future_layout.addWidget(future_label, 0, Qt.AlignCenter)
            
            # 连接停止按钮的点击事件
            future_btn_right.clicked.connect(self.stopDroneSystem)
            
            function_layout.addWidget(future_btn_right, 1, 2)
            
            # 设置列和行的拉伸因子，使布局更合理
            function_layout.setColumnStretch(0, 1)  # 左列
            function_layout.setColumnStretch(1, 4)  # 中列
            function_layout.setColumnStretch(2, 1)  # 右列
            function_layout.setRowStretch(0, 1)     # 上行
            function_layout.setRowStretch(1, 4)     # 中行
            function_layout.setRowStretch(2, 1)     # 下行
            
            # 设置按钮之间的对齐方式和间距
            function_layout.setAlignment(return_home_btn, Qt.AlignCenter)
            function_layout.setAlignment(explore_btn, Qt.AlignCenter)
            function_layout.setAlignment(start_btn, Qt.AlignCenter)
            function_layout.setAlignment(future_btn_right, Qt.AlignCenter)
            
            # 将网格布局添加到容器中
            function_container.addWidget(function_grid)
        
        # 添加功能区到功能组
        function_group_layout.addWidget(function_area)
        
        # 添加功能组到左侧边栏，减少拉伸系数给状态组更多空间
        left_sidebar_layout.addWidget(function_group, 2)  # 减少拉伸系数，让状态组有更多空间
        
        # 添加左侧边栏和RViz显示区域到分割器
        self.main_splitter.addWidget(self.left_sidebar)
        
        # 创建侧边栏控制按钮容器
        sidebar_control_container = QWidget()
        sidebar_control_container.setFixedWidth(20)  # 增加宽度到20px
        sidebar_control_container.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
        sidebar_control_container.setStyleSheet("""
            QWidget {
                background-color: #1A202C;
                border-left: none;
                border-right: 1px solid #3498DB;
            }
        """)
        
        # 创建垂直布局
        sidebar_control_layout = QVBoxLayout(sidebar_control_container)
        sidebar_control_layout.setContentsMargins(0, 0, 0, 0)
        sidebar_control_layout.setSpacing(0)
        
        # 创建切换按钮，使用图标替代文字
        self.toggle_sidebar_btn = QPushButton()
        self.toggle_sidebar_btn.setFixedWidth(20)  # 保持宽度
        self.toggle_sidebar_btn.setFixedHeight(50)  # 设置固定高度使图标更显眼
        # 使用图标
        self.toggle_sidebar_btn.setIcon(QIcon(":/images/icons/dropleft.svg"))
        self.toggle_sidebar_btn.setIconSize(QSize(16, 16))
        self.toggle_sidebar_btn.setStyleSheet("""
            QPushButton {
                background-color: #1A202C;  /* 与周围颜色相协调 */
                border: none;
                border-radius: 0;
                padding: 2px;
            }
            QPushButton:hover {
                background-color: #3498DB;  /* 蓝色悬停效果 */
            }
            QPushButton:pressed {
                background-color: #2980B9;  /* 按下效果 */
            }
        """)
        self.toggle_sidebar_btn.setCursor(Qt.PointingHandCursor)
        
        # 当按钮被点击时触发侧边栏的显示/隐藏或固定
        self.sidebar_expanded = True
        self.toggle_sidebar_btn.clicked.connect(self.toggleLeftSidebarPinned)
        
        # 将按钮添加到布局
        sidebar_control_layout.addWidget(self.toggle_sidebar_btn, 0, Qt.AlignCenter)
        
        # 添加控制容器到主分割器
        self.main_splitter.addWidget(sidebar_control_container)
        
        # 创建右侧分割器，只用于RViz显示区域
        self.right_splitter = QSplitter(Qt.Horizontal)
        
        # 添加RViz框架
        self.right_splitter.addWidget(self.frame)
        
        # # 设置按钮点击处理函数更新
        # self.settings_button.clicked.connect(self.toggleRVizDisplayPanel)
        
        # 添加右侧分割器到主分割器
        self.main_splitter.addWidget(self.right_splitter)
        
        # 创建右侧栏控制按钮容器
        right_sidebar_control_container = QWidget()
        right_sidebar_control_container.setFixedWidth(20)  # 增加宽度到20px
        right_sidebar_control_container.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
        right_sidebar_control_container.setStyleSheet("""
            QWidget {
                background-color: #1A202C;
                border-left: none;
                border-right: 1px solid #3498DB;
            }
        """)
        
        # 创建垂直布局
        right_sidebar_control_layout = QVBoxLayout(right_sidebar_control_container)
        right_sidebar_control_layout.setContentsMargins(0, 0, 0, 0)
        right_sidebar_control_layout.setSpacing(0)
        
        # 创建切换按钮，使用图标替代文字
        self.toggle_right_sidebar_btn = QPushButton()
        self.toggle_right_sidebar_btn.setFixedWidth(20)  # 保持宽度
        self.toggle_right_sidebar_btn.setFixedHeight(50)  # 设置固定高度使图标更显眼
        # 使用图标
        self.toggle_right_sidebar_btn.setIcon(QIcon(":/images/icons/dropright.svg"))
        self.toggle_right_sidebar_btn.setIconSize(QSize(16, 16))
        self.toggle_right_sidebar_btn.setStyleSheet("""
            QPushButton {
                background-color: #1A202C;  /* 与周围颜色相协调 */
                border: none;
                border-radius: 0;
                padding: 2px;
            }
            QPushButton:hover {
                background-color: #3498DB;  /* 蓝色悬停效果 */
            }
            QPushButton:pressed {
                background-color: #2980B9;  /* 按下效果 */
            }
        """)
        self.toggle_right_sidebar_btn.setCursor(Qt.PointingHandCursor)
        
        # 当按钮被点击时触发右侧栏的显示/隐藏或固定
        self.right_sidebar_expanded = True
        self.toggle_right_sidebar_btn.clicked.connect(self.toggleRightSidebarPinned)
        
        # 将按钮添加到布局
        right_sidebar_control_layout.addWidget(self.toggle_right_sidebar_btn, 0, Qt.AlignCenter)
        
        # 添加右侧控制按钮容器到主分割器
        self.main_splitter.addWidget(right_sidebar_control_container)
        
        # 创建右侧栏
        self.right_sidebar = QWidget()
        self.right_sidebar.setFixedWidth(self.adaptive_right_width)  # 使用自适应宽度
        # 使右侧栏可以在垂直方向调整大小
        self.right_sidebar.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)  # 设置固定宽度策略
        right_sidebar_layout = QVBoxLayout(self.right_sidebar)
        right_sidebar_layout.setContentsMargins(5, 5, 5, 5)  # 设置较小的边距
        right_sidebar_layout.setSpacing(0)  # 去除组件间距
        
        # 添加标题（已移除文本）
        image_title = QLabel("")
        image_title.setStyleSheet("padding: 0px;")
        image_title.setAlignment(Qt.AlignCenter)
        
        # 减少顶部弹性空间，让表格区域有更多空间
        right_sidebar_layout.addSpacing(10)
        
        # 添加待搜索人员位置窗口
        person_position_group = QGroupBox("📍 待搜索人员位置")
        person_position_group.setStyleSheet("""
            QGroupBox {
                color: #3498DB;
                font-size: 16pt;
                font-weight: bold;
                border: 2px solid #3498DB;
                border-radius: 12px;
                padding: 15px;
                margin-top: 20px;
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 rgba(52, 152, 219, 0.1),
                    stop:1 rgba(26, 32, 44, 0.8));
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top center;
                padding: 0 10px;
                background-color: #1E2330;
                border-radius: 6px;
            }
        """)  # 设置标题样式
        # 设置大小策略为垂直方向可扩展
        person_position_group.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        person_position_layout = QVBoxLayout(person_position_group)
        person_position_layout.setContentsMargins(0, 0, 0, 0)  # 进一步减少内边距
        person_position_layout.setSpacing(3)  # 进一步减少组件间距以节省空间
        
        # 创建位置显示区域
        position_frame = QFrame()
        position_frame.setFrameShape(QFrame.StyledPanel)
        position_frame.setStyleSheet("background-color: #1A202C; border-radius: 10px; border: 1px solid #3498DB;")
        # 设置Frame可扩展
        position_frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        position_frame_layout = QVBoxLayout(position_frame)
        position_frame_layout.setContentsMargins(0, 0, 0, 0)  # 进一步减少内边距
        
        # 创建位置信息表格
        self.position_table = QTableWidget()
        self.position_table.setColumnCount(5)  # 增加一列用于截图
        self.position_table.setHorizontalHeaderLabels(["ID", "X坐标", "Y坐标", "状态", "截图"])
        # 添加表格单元格点击事件
        self.position_table.cellClicked.connect(self.on_position_table_cell_clicked)
        self.position_table.setStyleSheet("""
            QTableWidget {
                background-color: #1E2330;
                color: white;
                gridline-color: #3498DB;
                border: none;
            }
            QHeaderView::section {
                background-color: #2C3E50;
                color: white;
                padding: 0px;
                border: 1px solid #3498DB;
            }
            QTableWidget::item {
                border-bottom: 1px solid #3498DB;
                padding: 0px;
            }
            QTableWidget::item:selected {
                background-color: #3498DB;
            }
        """)
        # 设置表格列宽策略，避免横向滚动条
        self.setupTableColumnWidths()
        self.position_table.verticalHeader().setVisible(False)
        # 设置表格可扩展，减少最小高度以更好填充空间
        self.position_table.setMinimumHeight(100)
        # 不设置最大高度限制，允许根据可用空间自动调整
        self.position_table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        # 禁用横向滚动条
        self.position_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        
        # 将表格初始化为空
        self.position_table.setRowCount(0)
        
        # 添加表格到位置框架
        position_frame_layout.addWidget(self.position_table)
        
        # 添加操作按钮区域
        button_container = QWidget()
        button_layout = QHBoxLayout(button_container)
        button_layout.setContentsMargins(0, 0, 0, 0)  # 减少顶部边距
        button_layout.setSpacing(3)  # 进一步减少按钮间距
        
        # 添加按钮
        add_btn = QPushButton("添加")
        add_btn.setStyleSheet("""
            QPushButton {
                background-color: #2980B9;
                color: white;
                border-radius: 4px;
                padding: 5px 15px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #3498DB;
            }
        """)
        add_btn.clicked.connect(self.addPerson)
        
        remove_btn = QPushButton("删除")
        remove_btn.setStyleSheet("""
            QPushButton {
                background-color: #C0392B;
                color: white;
                border-radius: 4px;
                padding: 5px 15px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #E74C3C;
            }
        """)
        remove_btn.clicked.connect(self.removePerson)
        
        update_btn = QPushButton("更新状态")
        update_btn.setStyleSheet("""
            QPushButton {
                background-color: #27AE60;
                color: white;
                border-radius: 4px;
                padding: 5px 15px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #2ECC71;
            }
        """)
        update_btn.clicked.connect(self.updatePersonStatus)
        
        button_layout.addWidget(add_btn)
        button_layout.addWidget(remove_btn)
        button_layout.addWidget(update_btn)
        
        # 添加按钮容器到位置框架
        position_frame_layout.addWidget(button_container)
        
        # 添加位置框架到位置组
        person_position_layout.addWidget(position_frame)
        
        # 添加位置组到右侧栏，并给予较大的拉伸系数
        right_sidebar_layout.addWidget(person_position_group, 2)  # 拉伸系数为2，表示会占用较多可用空间
        
        # 在底部添加图像显示区域和控制按钮
        image_display_container = QWidget()
        image_display_layout = QVBoxLayout(image_display_container)
        image_display_layout.setContentsMargins(0, 0, 0, 0)
        image_display_layout.setSpacing(0)  # 去除组件间距
        
        # 创建鸟瞰图显示区域 - 现代化卡片设计
        bird_view_group = QGroupBox("🗺️ 障碍物鸟瞰图")
        bird_view_group.setStyleSheet("""
            QGroupBox {
                color: #3498DB;
                font-size: 12pt;
                font-weight: bold;
                border: 2px solid #3498DB;
                border-radius: 8px;
                padding: 8px;
                margin-top: 15px;
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 rgba(52, 152, 219, 0.1),
                    stop:1 rgba(26, 32, 44, 0.8));
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top center;
                padding: 0 8px;
                background-color: #1E2330;
                border-radius: 4px;
            }
        """)
        bird_view_layout = QVBoxLayout(bird_view_group)
        bird_view_layout.setContentsMargins(0, 0, 0, 0)  # 进一步减少内边距
        bird_view_layout.setSpacing(0)

        # 鸟瞰图显示容器
        bird_view_frame = QFrame()
        bird_view_frame.setFrameShape(QFrame.StyledPanel)
        bird_view_frame.setStyleSheet("""
            QFrame {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 rgba(44, 62, 80, 0.9),
                    stop:1 rgba(26, 32, 44, 0.9));
                border-radius: 6px;
                border: 1px solid rgba(52, 152, 219, 0.3);
            }
        """)
        bird_view_frame_layout = QVBoxLayout(bird_view_frame)
        bird_view_frame_layout.setContentsMargins(0, 0, 0, 0)  # 进一步减少内边距
        bird_view_frame_layout.setSpacing(0)

        # 鸟瞰图显示
        self.bird_view_label = QLabel()
        self.bird_view_label.setAlignment(Qt.AlignCenter)
        self.bird_view_label.setFixedSize(self.adaptive_image_width - 16, self.adaptive_bird_height - 10)  # 调整尺寸适应新布局
        self.bird_view_label.setStyleSheet("""
            QLabel {
                background-color: #1A202C;
                border: 1px solid #3498DB;
                border-radius: 4px;
                color: #FFFFFF;
                font-size: 10pt;
            }
        """)
        self.bird_view_label.setText("等待鸟瞰图数据...")
        self.bird_view_label.setScaledContents(True)  # 启用内容缩放
        bird_view_frame_layout.addWidget(self.bird_view_label, 0, Qt.AlignCenter)

        bird_view_layout.addWidget(bird_view_frame)
        
        # 添加鸟瞰图组到图像显示容器
        image_display_layout.addWidget(bird_view_group)
        
        # 添加一个小间隔
        spacer = QWidget()
        spacer.setFixedHeight(8)
        image_display_layout.addWidget(spacer)

        # 创建图像显示组 - 现代化卡片设计
        image_group = QGroupBox("📷 实时图像")
        image_group.setStyleSheet("""
            QGroupBox {
                color: #3498DB;
                font-size: 12pt;
                font-weight: bold;
                border: 2px solid #3498DB;
                border-radius: 8px;
                padding: 8px;
                margin-top: 15px;
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 rgba(52, 152, 219, 0.1),
                    stop:1 rgba(26, 32, 44, 0.8));
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top center;
                padding: 0 8px;
                background-color: #1E2330;
                border-radius: 4px;
            }
        """)
        image_group_layout = QVBoxLayout(image_group)
        image_group_layout.setContentsMargins(0, 0, 0, 0)  # 进一步减少内边距
        image_group_layout.setSpacing(0)  # 进一步减少组件间距

        # 创建按钮区域 - 现代化切换按钮
        button_container = QFrame()
        button_container.setFrameShape(QFrame.StyledPanel)
        button_container.setStyleSheet("""
            QFrame {
                background-color: #2C3E50;
                border-radius: 6px;
                border: 1px solid rgba(52, 152, 219, 0.3);
            }
        """)
        button_layout = QHBoxLayout(button_container)
        button_layout.setContentsMargins(0, 0, 0, 0)  # 进一步减少内边距
        button_layout.setSpacing(0)  # 进一步减少按钮间距

        # RGB图像按钮 - 现代化设计
        self.rgb_button = QPushButton("RGB图像")
        self.rgb_button.setCheckable(True)
        self.rgb_button.setChecked(True)
        self.rgb_button.clicked.connect(self.switchToRGBImage)
        self.rgb_button.setStyleSheet("""
            QPushButton {
                background-color: #27AE60;
                color: white;
                border: none;
                border-radius: 4px;
                padding: 6px 12px;
                font-weight: bold;
                font-size: 10pt;
                min-height: 25px;
            }
            QPushButton:hover {
                background-color: #2ECC71;
            }
            QPushButton:pressed {
                background-color: #229954;
            }
            QPushButton:checked {
                background-color: #1ABC9C;
                border: 1px solid #16A085;
            }
        """)
        button_layout.addWidget(self.rgb_button)

        # 深度图像按钮 - 现代化设计
        self.depth_button = QPushButton("深度图像")
        self.depth_button.setCheckable(True)
        self.depth_button.clicked.connect(self.switchToDepthImage)
        self.depth_button.setStyleSheet("""
            QPushButton {
                background-color: #34495E;
                color: white;
                border: none;
                border-radius: 4px;
                padding: 6px 12px;
                font-weight: bold;
                font-size: 10pt;
                min-height: 25px;
            }
            QPushButton:hover {
                background-color: #3498DB;
            }
            QPushButton:pressed {
                background-color: #2980B9;
            }
            QPushButton:checked {
                background-color: #1ABC9C;
                border: 1px solid #16A085;
            }
        """)
        button_layout.addWidget(self.depth_button)

        image_group_layout.addWidget(button_container)

        # 创建图像显示容器
        image_frame = QFrame()
        image_frame.setFrameShape(QFrame.StyledPanel)
        image_frame.setStyleSheet("""
            QFrame {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 rgba(44, 62, 80, 0.9),
                    stop:1 rgba(26, 32, 44, 0.9));
                border-radius: 6px;
                border: 1px solid rgba(52, 152, 219, 0.3);
            }
        """)
        image_frame_layout = QVBoxLayout(image_frame)
        image_frame_layout.setContentsMargins(2, 2, 2, 2)  # 进一步减少内边距
        image_frame_layout.setSpacing(0)

        # 图像显示标签
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setFixedSize(self.adaptive_image_width - 16, self.adaptive_image_height - 10)  # 调整尺寸适应新布局
        self.image_label.setStyleSheet("""
            QLabel {
                background-color: #1A202C;
                border: 1px solid #3498DB;
                border-radius: 4px;
                color: #FFFFFF;
                font-size: 10pt;
            }
        """)
        self.image_label.setText("""
            <div style='
                display: flex;
                align-items: center;
                justify-content: center;
                height: 100%;
                width: 100%;
                font-size: 16pt;
                color: #3498DB;
                text-align: center;
                font-weight: bold;
            '>
                等待图像...
            </div>
        """)
        self.image_label.setScaledContents(True)  # 启用内容缩放
        image_frame_layout.addWidget(self.image_label, 0, Qt.AlignCenter)

        image_group_layout.addWidget(image_frame)

        # 添加图像组到图像显示布局
        image_display_layout.addWidget(image_group)
        
        # 添加图像显示容器，使用拉伸系数1
        right_sidebar_layout.addWidget(image_display_container, 1)
        
        # 设置当前图像类型
        self.current_image_mode = "rgb"  # 默认显示RGB图像
        
        # 深度图像数据
        self.depth_image = None
        
        # 鸟瞰图数据
        self.bird_view_image = None

        # 添加右侧栏到主分割器
        self.main_splitter.addWidget(self.right_sidebar)
        
        # 设置分割器手柄宽度
        self.main_splitter.setHandleWidth(3)  # 设置较小的分割器手柄宽度
        self.right_splitter.setHandleWidth(3)  # 设置较小的分割器手柄宽度
        self.main_splitter.setChildrenCollapsible(False)  # 防止子部件被完全折叠
        self.right_splitter.setChildrenCollapsible(False)  # 防止子部件被完全折叠
        
        # 设置自适应的分割器初始比例
        self.setupAdaptiveSplitterSizes()

        # 设置窗口最小尺寸，确保在小屏幕上也能正常显示
        min_width = self.adaptive_left_width + self.adaptive_right_width + 500  # 最小中间区域500px
        min_height = 600  # 最小高度600px
        self.setMinimumSize(min_width, min_height)

        # 禁止分割器伸缩右侧栏
        self.main_splitter.setStretchFactor(0, 0)  # 左侧栏不自动拉伸
        self.main_splitter.setStretchFactor(1, 0)  # 左侧控制按钮不自动拉伸
        self.main_splitter.setStretchFactor(2, 1)  # 中间RViz区域自动拉伸
        self.main_splitter.setStretchFactor(3, 0)  # 右侧控制按钮不自动拉伸
        self.main_splitter.setStretchFactor(4, 0)  # 右侧栏不自动拉伸
        
        # 初始分割比例将在setupAdaptiveSplitterSizes中设置
        
        main_layout.addWidget(self.main_splitter)
        
        # 创建底部状态栏，包含位置和时间信息
        status_bar = QStatusBar()
        status_bar.setStyleSheet("background-color: #1A202C; padding: 2px;")
        status_bar.setMaximumHeight(25)  # 限制底部状态栏高度
        
        # 创建位置显示标签(左侧)
        self.position_label = QLabel("Position: (X:0.0000 Y:0.0000 Z:0.0000)")
        self.position_label.setStyleSheet("color: #3498DB; padding-left: 15px; font-weight: bold;")
        self.position_label.setMinimumWidth(300)  # 设置最小宽度确保显示完整
        status_bar.addWidget(self.position_label)
        
        # 添加占位符，使FPS和时间显示在右侧
        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        status_bar.addWidget(spacer)
        
        # 创建帧率标签(英文)
        self.frame_rate_label = QLabel("FPS: 0.00")
        self.frame_rate_label.setStyleSheet("color: #3498DB; padding-right: 15px;")
        status_bar.addPermanentWidget(self.frame_rate_label)
        
        # 创建ROS时间显示(英文)
        self.ros_time_label = QLabel("Time: 0.0000")
        self.ros_time_label.setStyleSheet("color: #3498DB; padding-right: 50px;")  # 增加右侧内边距，避免与右侧按钮重叠
        status_bar.addPermanentWidget(self.ros_time_label)
        
        # 注意：定时器已在_init_timers中统一初始化，这里不再重复创建

        # 记录帧率计算的变量
        self.frame_count = 0
        self.last_frame_time = time.time()

        # 速度数据
        self.speed = 0
        self.linear_speed = 0
        self.angular_speed = 0

        # 初始状态设置为未连接
        if hasattr(self, 'connection_label'):
            self.connection_label.setText("未连接")
            # 不设置字体大小，保持卡片的原始字体设置
            self.connection_label.setStyleSheet("""
                QLabel {
                    color: #E74C3C;
                    font-weight: bold;
                    background: transparent;
                    border: none;
                    padding: 0px;
                    margin: 0px;
                }
            """)
        if hasattr(self, 'mode_label'):
            self.mode_label.setText("未连接")
        
        main_layout.addWidget(status_bar)
        
        # 设置布局的拉伸因子，让中间的RViz显示区域占据大部分空间
        main_layout.setStretch(1, 10)  # 中间部分(splitter)占据更多空间
        main_layout.setStretch(0, 1)   # 标题栏占据较少空间
        main_layout.setStretch(2, 1)   # 底部栏占据较少空间

        # QMainWindow已经在初始化时设置了central_widget，不需要再调用setLayout
        
        # 注意：延迟初始化已在_init_timers中处理，避免重复
        
        # 初始化话题订阅器变量，但不启动订阅（在__init__末尾会自动订阅）
        self.topic_subscriber = None
        
        # 用于存储小球截图的字典 {ball_id: {"path": 文件路径, "timestamp": 时间戳}}
        # 注意：不再存储图像数据，只存储文件路径以避免内存问题
        
        # 截图目录已在初始化时设置，这里不需要重复设置

    def calculateAdaptiveSizes(self):
        """根据屏幕分辨率计算自适应尺寸"""
        # 基准分辨率为1920x1080
        base_width = 1920
        base_height = 1080

        # 计算自适应的侧边栏宽度 - 使用固定比例而不是绝对值
        # 左侧栏占屏幕宽度的比例：小屏幕20%，中等屏幕22%，大屏幕25%
        # 右侧栏占屏幕宽度的比例：小屏幕25%，中等屏幕28%，大屏幕30%

        if self.screen_width <= 1366:  # 小屏幕 (1366x768等)
            left_ratio = 0.20
            right_ratio = 0.25
        elif self.screen_width <= 1920:  # 中等屏幕 (1920x1080)
            left_ratio = 0.22
            right_ratio = 0.28
        else:  # 大屏幕 (2K, 4K等)
            left_ratio = 0.25
            right_ratio = 0.30

        # 计算侧边栏宽度，设置合理的最小值和最大值
        self.adaptive_left_width = max(280, min(600, int(self.screen_width * left_ratio)))
        self.adaptive_right_width = max(350, min(800, int(self.screen_width * right_ratio)))

        # 初始化图像尺寸变量，这些将在updateImageSizes中动态计算
        self.adaptive_image_width = 320
        self.adaptive_image_height = 240
        self.adaptive_bird_height = 120

        print(f"自适应尺寸 - 左侧栏: {self.adaptive_left_width}px, 右侧栏: {self.adaptive_right_width}px")

    def updateImageSizes(self):
        """动态更新图像尺寸以适应右侧栏宽度"""
        if not hasattr(self, 'right_sidebar'):
            return

        # 获取右侧栏的实际宽度
        actual_right_width = self.right_sidebar.width() if self.right_sidebar.isVisible() else self.adaptive_right_width

        # 计算图像区域可用宽度，留出边距
        margin = 20  # 减少边距以更好利用空间
        available_width = actual_right_width - margin

        # 确保最小宽度
        min_image_width = 240
        available_width = max(min_image_width, available_width)

        # 保持640:480的宽高比 (4:3)
        self.adaptive_image_width = available_width
        self.adaptive_image_height = int(available_width * 480 / 640)  # 640:480比例

        # 鸟瞰图高度为主图像高度的一半，但保持合理比例
        self.adaptive_bird_height = max(80, int(self.adaptive_image_height * 0.5))

        # 检查总高度是否合理
        available_height = self.screen_height - 300  # 预留300px给其他UI元素
        total_image_height = self.adaptive_image_height + self.adaptive_bird_height + 100  # 100px给按钮和间距

        if total_image_height > available_height:
            # 按比例缩小
            scale_factor = available_height / total_image_height
            self.adaptive_image_height = int(self.adaptive_image_height * scale_factor)
            self.adaptive_bird_height = int(self.adaptive_bird_height * scale_factor)
            # 根据高度重新计算宽度，保持640:480比例
            self.adaptive_image_width = int(self.adaptive_image_height * 640 / 480)

        # 更新图像组件尺寸
        if hasattr(self, 'image_label'):
            self.image_label.setFixedSize(self.adaptive_image_width, self.adaptive_image_height)
        if hasattr(self, 'bird_view_label'):
            self.bird_view_label.setFixedSize(self.adaptive_image_width, self.adaptive_bird_height)
        if hasattr(self, 'rgb_button') and hasattr(self, 'depth_button'):
            button_width = self.adaptive_image_width // 2
            # 更新按钮样式以适应新宽度
            self.updateButtonStyles(button_width)

        print(f"更新图像尺寸 - 图像: {self.adaptive_image_width}x{self.adaptive_image_height}px, 鸟瞰图: {self.adaptive_image_width}x{self.adaptive_bird_height}px")

        # 同时更新表格列宽
        if hasattr(self, 'position_table'):
            self.setupTableColumnWidths()

    def setupTableColumnWidths(self):
        """设置表格列宽，前4列等宽，最后一列拉伸填充"""
        if not hasattr(self, 'position_table') or not hasattr(self, 'right_sidebar'):
            return

        # 获取右侧栏的实际宽度
        sidebar_width = self.right_sidebar.width() if self.right_sidebar.isVisible() else self.adaptive_right_width

        # 计算表格可用宽度（减去边距、边框和滚动条）
        available_width = sidebar_width - 50  # 减去左右边距、边框和可能的滚动条

        # 计算前4列的统一宽度
        # 为最后一列预留合理宽度，剩余空间平均分配给前4列
        min_last_column_width = 80   # 截图列最小宽度
        max_last_column_width = 150  # 截图列最大宽度，避免过度拉伸

        # 计算理想的最后一列宽度
        ideal_last_column_width = min(max_last_column_width, max(min_last_column_width, available_width * 0.25))

        # 计算前4列可用的总宽度
        width_for_first_4_columns = available_width - ideal_last_column_width

        # 确保有足够空间
        if width_for_first_4_columns < 200:  # 如果空间太小
            # 紧凑模式：前4列使用更小的统一宽度
            uniform_width = max(35, width_for_first_4_columns // 4)
            # 重新计算最后一列宽度
            actual_last_column_width = available_width - uniform_width * 4
            min_last_column_width = max(60, actual_last_column_width)
        else:
            # 正常模式：前4列使用合适的统一宽度
            uniform_width = max(50, min(75, width_for_first_4_columns // 4))
            # 重新计算实际的最后一列宽度
            actual_last_column_width = available_width - uniform_width * 4

        # 设置列宽数组：前4列等宽，最后一列拉伸
        column_widths = [uniform_width] * 4  # 前4列：ID, X坐标, Y坐标, 状态

        # 设置各列的宽度和调整模式
        header = self.position_table.horizontalHeader()

        # 前4列设置为固定宽度且等宽
        for i in range(4):
            header.setSectionResizeMode(i, QHeaderView.Fixed)
            self.position_table.setColumnWidth(i, uniform_width)

        # 最后一列（截图列）设置为拉伸模式，自动填充剩余空间
        header.setSectionResizeMode(4, QHeaderView.Stretch)

        # 设置表格头的最小截面大小，避免过度压缩
        header.setMinimumSectionSize(min_last_column_width)

        print(f"表格列宽设置: 可用宽度={available_width}px, 前4列统一宽度={uniform_width}px, 最后列宽度={actual_last_column_width}px (范围:{min_last_column_width}-{max_last_column_width}px)")

    def updateButtonStyles(self, button_width):
        """更新按钮样式以适应新宽度 - 现在使用内联样式，此函数保留以兼容性"""
        # 按钮样式现在在创建时直接设置，无需动态更新
        pass

    def setupAdaptiveSplitterSizes(self):
        """设置自适应的分割器尺寸"""
        # 使用定时器延迟设置，确保窗口已经完全初始化
        QTimer.singleShot(100, self._setAdaptiveSplitterSizes)

    def _setAdaptiveSplitterSizes(self):
        """实际设置分割器尺寸的方法"""
        try:
            # 获取当前窗口宽度，如果窗口还没有显示，使用屏幕宽度
            current_width = self.width() if self.isVisible() else self.screen_width

            # 计算各部分的宽度
            control_button_width = 20  # 控制按钮宽度
            total_control_width = control_button_width * 2  # 两个控制按钮

            # 检查侧边栏当前是否可见
            left_visible = hasattr(self, 'left_sidebar') and self.left_sidebar.isVisible()
            right_visible = hasattr(self, 'right_sidebar') and self.right_sidebar.isVisible()

            # 根据侧边栏可见性计算实际使用的宽度
            actual_left_width = self.adaptive_left_width if left_visible else 0
            actual_right_width = self.adaptive_right_width if right_visible else 0

            # 计算中间RViz区域的宽度
            remaining_width = current_width - actual_left_width - actual_right_width - total_control_width

            # 确保中间区域有最小宽度
            min_center_width = 400
            if remaining_width < min_center_width:
                # 如果空间不足，按比例缩小侧边栏
                total_sidebar_width = actual_left_width + actual_right_width
                available_sidebar_width = current_width - min_center_width - total_control_width

                if available_sidebar_width > 0 and total_sidebar_width > 0:
                    scale_factor = available_sidebar_width / total_sidebar_width
                    adjusted_left_width = max(200 if left_visible else 0, int(actual_left_width * scale_factor))
                    adjusted_right_width = max(300 if right_visible else 0, int(actual_right_width * scale_factor))
                    remaining_width = current_width - adjusted_left_width - adjusted_right_width - total_control_width
                else:
                    # 极端情况，使用最小值
                    adjusted_left_width = 200 if left_visible else 0
                    adjusted_right_width = 300 if right_visible else 0
                    remaining_width = max(min_center_width, current_width - adjusted_left_width - adjusted_right_width - total_control_width)
            else:
                adjusted_left_width = actual_left_width
                adjusted_right_width = actual_right_width

            # 设置分割器尺寸
            sizes = [adjusted_left_width, control_button_width, remaining_width, control_button_width, adjusted_right_width]

            # 只有在尺寸真正改变时才设置，避免不必要的布局调整
            current_sizes = self.main_splitter.sizes()
            if current_sizes != sizes:
                self.main_splitter.setSizes(sizes)
                print(f"分割器尺寸设置: 左侧栏={adjusted_left_width}px, 中间={remaining_width}px, 右侧栏={adjusted_right_width}px")

        except Exception as e:
            print(f"设置分割器尺寸时出错: {str(e)}")
    
    def createStatusCards(self, parent_layout):
        """创建现代化的状态卡片"""
        # 第一行状态卡片
        row1_container = QWidget()
        row1_layout = QHBoxLayout(row1_container)
        row1_layout.setContentsMargins(0, 0, 0, 0)
        row1_layout.setSpacing(3)  # 进一步减少卡片间距

        # 飞行模式卡片 - 使用compact模式
        mode_card = self.createStatusCard("飞行模式", "MANUAL", "#3498DB", compact=True)
        self.mode_label = mode_card.findChild(QLabel, "value_label")
        row1_layout.addWidget(mode_card)

        # 连接状态卡片 - 使用compact模式
        connection_card = self.createStatusCard("连接状态", "已连接", "#2ECC71", compact=True)
        self.connection_label = connection_card.findChild(QLabel, "value_label")
        row1_layout.addWidget(connection_card)

        parent_layout.addWidget(row1_container)

        # 第二行状态卡片
        row2_container = QWidget()
        row2_layout = QHBoxLayout(row2_container)
        row2_layout.setContentsMargins(0, 0, 0, 0)
        row2_layout.setSpacing(3)  # 进一步减少卡片间距

        # 飞行高度卡片 - 使用compact模式
        altitude_card = self.createStatusCard("飞行高度", "0.0000 m", "#E67E22", compact=True)
        self.altitude_label = altitude_card.findChild(QLabel, "value_label")
        row2_layout.addWidget(altitude_card)

        # 地面速度卡片 - 使用compact模式
        speed_card = self.createStatusCard("地面速度", "0.0000 m/s", "#9B59B6", compact=True)
        self.ground_speed_label = speed_card.findChild(QLabel, "value_label")
        row2_layout.addWidget(speed_card)

        parent_layout.addWidget(row2_container)

        # 第三行状态卡片 - 姿态信息
        row3_container = QWidget()
        row3_layout = QHBoxLayout(row3_container)
        row3_layout.setContentsMargins(0, 0, 0, 0)
        row3_layout.setSpacing(3)  # 进一步减少卡片间距

        # 俯仰角卡片
        pitch_card = self.createStatusCard("俯仰角", "0.00°", "#1ABC9C", compact=True)
        self.pitch_label = pitch_card.findChild(QLabel, "value_label")
        row3_layout.addWidget(pitch_card)

        # 滚转角卡片
        roll_card = self.createStatusCard("滚转角", "0.00°", "#F39C12", compact=True)
        self.roll_label = roll_card.findChild(QLabel, "value_label")
        row3_layout.addWidget(roll_card)

        # 偏航角卡片
        yaw_card = self.createStatusCard("偏航角", "0.00°", "#E74C3C", compact=True)
        self.yaw_label = yaw_card.findChild(QLabel, "value_label")
        row3_layout.addWidget(yaw_card)

        parent_layout.addWidget(row3_container)

        # 第四行 - 电池状态卡片（全宽）
        battery_card = self.createStatusCard("电池状态", f"{self.battery_percentage:.1f}%", "#27AE60", full_width=True)
        self.battery_status_label = battery_card.findChild(QLabel, "value_label")
        parent_layout.addWidget(battery_card)

    def createStatusCard(self, title, value, color, compact=False, full_width=False):
        """创建单个状态卡片"""
        card = QFrame()
        card.setFrameShape(QFrame.StyledPanel)

        # 根据是否紧凑模式和全宽模式设置不同的样式
        if compact:
            card.setStyleSheet(f"""
                QFrame {{
                    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                        stop:0 rgba(255, 255, 255, 0.08),
                        stop:1 rgba(0, 0, 0, 0.12));
                    border: 1px solid {color};
                    border-radius: 6px;
                    margin: 0px;
                }}
                QFrame:hover {{
                    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                        stop:0 rgba(255, 255, 255, 0.12),
                        stop:1 rgba(0, 0, 0, 0.08));
                    border: 2px solid {color};
                }}
            """)
            # 根据屏幕分辨率调整高度，增加高度以更好填充空间
            if hasattr(self, 'screen_height') and self.screen_height <= 768:  # 1K分辨率或更小
                card.setMinimumHeight(60)
                card.setMaximumHeight(60)
            else:
                card.setMinimumHeight(70)
                card.setMaximumHeight(70)
        else:
            card.setStyleSheet(f"""
                QFrame {{
                    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                        stop:0 rgba(255, 255, 255, 0.08),
                        stop:1 rgba(0, 0, 0, 0.12));
                    border: 1px solid {color};
                    border-radius: 10px;
                    margin: 1px;
                }}
                QFrame:hover {{
                    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                        stop:0 rgba(255, 255, 255, 0.12),
                        stop:1 rgba(0, 0, 0, 0.08));
                    border: 2px solid {color};
                }}
            """)
            if full_width:
                card.setMinimumHeight(80)  # 增加全宽卡片高度
                card.setMaximumHeight(80)
            else:
                card.setMinimumHeight(90)  # 增加普通卡片高度
                card.setMaximumHeight(90)

        layout = QVBoxLayout(card)
        layout.setContentsMargins(0, 0, 0, 0)  # 去掉内边距
        layout.setSpacing(0)  # 去掉间距

        # 标题标签 - 占50%高度
        title_label = QLabel(title)

        # 根据屏幕分辨率和compact模式调整标题字体大小
        if compact:
            if hasattr(self, 'screen_height') and self.screen_height <= 768:  # 1K分辨率或更小
                title_font_size = '8pt'
            else:
                title_font_size = '9pt'
        else:
            title_font_size = '11pt'

        title_label.setStyleSheet(f"""
            QLabel {{
                color: #FFFFFF;
                font-size: {title_font_size};
                font-weight: normal;
                background: transparent;
                border: none;
                padding: 0px;
                margin: 0px;
            }}
        """)
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(title_label, 1)  # 拉伸因子1，占50%

        # 数值标签 - 占50%高度
        value_label = QLabel(value)
        value_label.setObjectName("value_label")  # 设置对象名以便查找

        # 根据屏幕分辨率和compact模式调整字体大小
        if compact:
            if hasattr(self, 'screen_height') and self.screen_height <= 768:  # 1K分辨率或更小
                font_size = '10pt'
            else:
                font_size = '11pt'
        else:
            font_size = '14pt'

        value_label.setStyleSheet(f"""
            QLabel {{
                color: {color};
                font-size: {font_size};
                font-weight: bold;
                background: transparent;
                border: none;
                padding: 0px;
                margin: 0px;
            }}
        """)
        value_label.setAlignment(Qt.AlignCenter)
        value_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(value_label, 1)  # 拉伸因子1，占50%

        return card

    def toggleDisplayPanel(self):
        """此方法已不再使用，保留以避免可能的引用错误"""
        self.toggleRVizDisplayPanel()
    
    def toggleLeftSidebarPinned(self):
        """切换左侧栏的固定状态"""
        if self.left_sidebar_pinned:
            # 如果当前是固定状态，解除固定并隐藏
            self.left_sidebar_pinned = False
            self.toggleSidebar(hide=True, animate=True)
            # 更新按钮样式，恢复正常
            self.toggle_sidebar_btn.setStyleSheet("""
                QPushButton {
                    background-color: #1A202C;  /* 与周围颜色相协调 */
                    border: none;
                    border-radius: 0;
                    padding: 2px;
                }
                QPushButton:hover {
                    background-color: #3498DB;  /* 蓝色悬停效果 */
                }
                QPushButton:pressed {
                    background-color: #2980B9;  /* 按下效果 */
                }
            """)
        else:
            # 如果当前非固定，切换为固定状态并显示
            self.left_sidebar_pinned = True
            self.toggleSidebar(hide=False, animate=True)
            # 更新按钮样式，显示固定状态
            self.toggle_sidebar_btn.setStyleSheet("""
                QPushButton {
                    background-color: #3498DB;  /* 蓝色背景表示已固定 */
                    border: none;
                    border-radius: 0;
                    padding: 2px;
                }
                QPushButton:hover {
                    background-color: #2980B9;
                }
                QPushButton:pressed {
                    background-color: #2980B9;
                }
            """)
    
    def toggleSidebar(self, hide=None, animate=False):
        """显示或隐藏侧边栏
        
        参数:
            hide: 是否隐藏侧边栏。如果为None，则切换当前状态
            animate: 是否使用动画效果
        """
        # 如果指定了hide参数，则根据参数决定是否隐藏
        should_hide = hide if hide is not None else self.sidebar_expanded
        
        # 如果已经在动画中，则不重复触发
        if hasattr(self, 'sidebar_animation') and self.sidebar_animation.state() == QPropertyAnimation.Running:
            return

        # 清理之前的动画对象
        if hasattr(self, 'sidebar_animation'):
            try:
                self.sidebar_animation.finished.disconnect()
                self.sidebar_animation.valueChanged.disconnect()
                self.sidebar_animation.stop()
                self.sidebar_animation.deleteLater()
            except:
                pass  # 忽略断开连接时的错误

        # 打印调试信息
        # print(f"切换侧边栏: hide={should_hide}, animate={animate}, 当前状态={self.sidebar_expanded}")

        if should_hide:
            # 隐藏侧边栏
            if animate:
                # 使用动画效果
                self.sidebar_animation = QPropertyAnimation(self.left_sidebar, b"maximumWidth")
                self.sidebar_animation.setDuration(200)  # 动画持续时间200ms
                current_width = self.left_sidebar.width()
                self.sidebar_animation.setStartValue(current_width)
                self.sidebar_animation.setEndValue(0)
                self.sidebar_animation.setEasingCurve(QEasingCurve.InOutQuad)

                # 确保侧边栏可见性正确
                self.left_sidebar.setVisible(True)

                # 动画结束后更新状态
                self.sidebar_animation.finished.connect(lambda: self.finishSidebarAnimation(False))

                # 动画过程中定期更新悬浮窗口位置
                self.sidebar_animation.valueChanged.connect(lambda: self._update_overlay_positions())

                # 启动动画
                self.sidebar_animation.start()
                
                # 立即更新状态，但不隐藏侧边栏（等动画完成）
                self.updateSidebarState(False)
                
                # 更新分割器尺寸
                sizes = self.main_splitter.sizes()
                self.main_splitter.setSizes([0, 20, sizes[0] + sizes[2]])
            else:
                # 直接隐藏
                self.left_sidebar.setMaximumWidth(0)
                self.left_sidebar.setMinimumWidth(0)
                self.left_sidebar.setVisible(False)
                self.updateSidebarState(False)
                
                # 更新分割器尺寸
                sizes = self.main_splitter.sizes()
                self.main_splitter.setSizes([0, 20, sizes[0] + sizes[2]])

                # 立即更新悬浮窗口位置
                QTimer.singleShot(50, self._update_overlay_positions)
        else:
            # 显示侧边栏
            if animate:
                # 清理之前的动画对象（如果存在）
                if hasattr(self, 'sidebar_animation'):
                    try:
                        self.sidebar_animation.finished.disconnect()
                        self.sidebar_animation.valueChanged.disconnect()
                        self.sidebar_animation.stop()
                        self.sidebar_animation.deleteLater()
                    except:
                        pass  # 忽略断开连接时的错误

                # 先设置最大宽度，以便动画可以工作
                self.left_sidebar.setMaximumWidth(self.adaptive_left_width)
                self.left_sidebar.setMinimumWidth(0)
                self.left_sidebar.setVisible(True)

                # 使用动画效果
                self.sidebar_animation = QPropertyAnimation(self.left_sidebar, b"maximumWidth")
                self.sidebar_animation.setDuration(200)  # 动画持续时间200ms
                self.sidebar_animation.setStartValue(0)
                self.sidebar_animation.setEndValue(self.adaptive_left_width)
                self.sidebar_animation.setEasingCurve(QEasingCurve.InOutQuad)

                # 动画结束后更新状态
                self.sidebar_animation.finished.connect(lambda: self.finishSidebarAnimation(True))

                # 动画过程中定期更新悬浮窗口位置
                self.sidebar_animation.valueChanged.connect(lambda: self._update_overlay_positions())

                # 启动动画
                self.sidebar_animation.start()

                # 立即更新状态
                self.updateSidebarState(True)

                # 更新分割器尺寸
                sizes = self.main_splitter.sizes()
                self.main_splitter.setSizes([self.adaptive_left_width, 20, sizes[2] - self.adaptive_left_width])
            else:
                # 直接显示
                self.left_sidebar.setFixedWidth(self.adaptive_left_width)  # 使用自适应宽度
                self.left_sidebar.setVisible(True)
                self.updateSidebarState(True)

                # 更新分割器尺寸
                sizes = self.main_splitter.sizes()
                self.main_splitter.setSizes([self.adaptive_left_width, 20, sizes[2] - self.adaptive_left_width])

                # 立即更新悬浮窗口位置
                QTimer.singleShot(50, self._update_overlay_positions)
    
    def finishSidebarAnimation(self, expanded):
        """动画结束后的处理

        参数:
            expanded: 是否展开
        """
        if not expanded:
            # 动画结束后，如果是隐藏状态，则设置不可见以减少资源占用
            self.left_sidebar.setVisible(False)
        else:
            # 如果是显示状态，确保最小宽度也设置好
            self.left_sidebar.setMinimumWidth(self.adaptive_left_width)

        # 动画完成后立即更新悬浮窗口位置，使用多次延迟更新确保位置正确
        self._update_overlay_positions()
        QTimer.singleShot(50, self._update_overlay_positions)
        QTimer.singleShot(100, self._update_overlay_positions)
        QTimer.singleShot(200, self._update_overlay_positions)
    
    def updateSidebarState(self, expanded):
        """更新侧边栏状态

        参数:
            expanded: 是否展开
        """
        self.sidebar_expanded = expanded
        self.toggle_sidebar_btn.setIcon(QIcon(":/images/icons/dropleft.svg" if expanded else ":/images/icons/dropright.svg"))
        self.toggle_sidebar_btn.style().unpolish(self.toggle_sidebar_btn)
        self.toggle_sidebar_btn.style().polish(self.toggle_sidebar_btn)

    def onTopButtonClick(self):
        self.switchToView("Top View")
        
    def onSideButtonClick(self):
        self.switchToView("Side View")
    
    def onFreeViewClick(self):
        view_man = self.manager.getViewManager()
        view_man.setCurrentViewControllerType("rviz/Orbit")
        
    def switchToView(self, view_name):
        view_man = self.manager.getViewManager()
        for i in range(view_man.getNumViews()):
            if view_man.getViewAt(i).getName() == view_name:
                view_man.setCurrentFrom(view_man.getViewAt(i))
                return
        print("Did not find view named %s." % view_name)

    def updateBatteryStatus(self, battery_data):
        """更新电池状态显示"""
        try:
            if not battery_data:
                return
                
            # 更新电池百分比和电压
            percentage = battery_data.get("percentage", 0.0) * 100  # 转换为百分比
            voltage = battery_data.get("voltage", 0.0)  # 获取电压值
            current = battery_data.get("current", 0.0)  # 获取电流值
            temperature = battery_data.get("temperature", 0.0)  # 获取温度
            
            # 保存数据以便在模拟模式下使用
            self.battery_percentage = percentage
            self.battery_voltage = voltage
            
            # 更新顶部状态栏电压显示
            if hasattr(self, 'voltage_label'):
                self.voltage_label.setText(f"{voltage:.2f} V")
            
            # 更新电池状态详情标签
            if hasattr(self, 'battery_status_label'):
                self.battery_status_label.setText(f"{percentage:.1f}% ({voltage:.2f}V)")
            
            # 根据电量选择对应图标并更新
            if hasattr(self, 'battery_icon_label'):
                if percentage <= 15:
                    icon_path = ":/images/icons/battery_0.svg"
                elif percentage <= 50:
                    icon_path = ":/images/icons/battery_50.svg"
                elif percentage <= 75:
                    icon_path = ":/images/icons/battery_75.svg"
                else:
                    icon_path = ":/images/icons/battery_100.svg"
                    
                # 更新电池图标
                self.battery_icon_label.setPixmap(QPixmap(icon_path).scaled(24, 24, Qt.KeepAspectRatio, Qt.SmoothTransformation))

            # 同时更新RViz悬浮窗口的电池和电压显示
            if hasattr(self, 'battery_value') and self.battery_value:
                try:
                    battery_formatted = f"{percentage:.1f}"
                    unit = self.battery_value.property("unit") or ""
                    self.battery_value.setText(f"{battery_formatted} {unit}".strip())
                except Exception as e:
                    pass  # 静默处理错误

            if hasattr(self, 'voltage_value') and self.voltage_value:
                try:
                    voltage_formatted = f"{voltage:.2f}"
                    unit = self.voltage_value.property("unit") or ""
                    self.voltage_value.setText(f"{voltage_formatted} {unit}".strip())
                except Exception as e:
                    pass  # 静默处理错误

            # 标记电池话题有数据
            self.topics_with_data["battery"] = True
            
        except Exception as e:
            print(f"更新电池状态显示时出错: {str(e)}")
    
    def updatePositionDisplay(self, odometry_data):
        """更新位置信息显示"""
        try:
            if not odometry_data:
                return
                
            # 获取位置数据
            pos_x = odometry_data["position"]["x"]
            pos_y = odometry_data["position"]["y"]
            pos_z = odometry_data["position"]["z"]
            
            # 更新状态栏位置标签，保留四位小数
            if hasattr(self, 'position_label'):
                self.position_label.setText(f"Position: (X:{pos_x:.4f} Y:{pos_y:.4f} Z:{pos_z:.4f})")
            
            # 更新高度显示
            if hasattr(self, 'altitude_label'):
                self.altitude_label.setText(f"{pos_z:.4f} m")

            # 同时更新RViz悬浮窗口的高度显示
            if hasattr(self, 'altitude_value') and self.altitude_value:
                try:
                    height_formatted = f"{pos_z:.2f}"
                    unit = self.altitude_value.property("unit") or ""
                    self.altitude_value.setText(f"{height_formatted} {unit}".strip())
                except Exception as e:
                    pass  # 静默处理错误

            # 标记位置话题有数据
            self.topics_with_data["odometry"] = True
            
        except Exception as e:
            print(f"更新位置显示时出错: {str(e)}")
    
    def updateVelocityDisplay(self, velocity_data):
        """更新速度信息显示"""
        try:
            if not velocity_data:
                return
                
            # 获取速度数据
            linear_x = velocity_data["linear"]["x"]
            linear_y = velocity_data["linear"]["y"]
            linear_z = velocity_data["linear"]["z"]
            
            # 获取角速度数据
            angular_x = velocity_data["angular"]["x"]
            angular_y = velocity_data["angular"]["y"]
            angular_z = velocity_data["angular"]["z"]
            
            # 计算合成速度(cm/s)
            speed = velocity_data.get("speed", 0.0)
            
            # 检查NaN值并替换为0
            if math.isnan(speed):
                speed = 0.0
                
            # 检查线性速度分量是否为NaN
            if math.isnan(linear_x):
                linear_x = 0.0
            if math.isnan(linear_y):
                linear_y = 0.0
            if math.isnan(linear_z):
                linear_z = 0.0
            
            # 保存速度数据以便在模拟模式下使用
            self.speed = int(speed)
            self.linear_speed = math.sqrt(linear_x**2 + linear_y**2)  # 地面速度
            
            # 更新地面速度标签
            if hasattr(self, 'ground_speed_label'):
                self.ground_speed_label.setText(f"{self.linear_speed:.4f} m/s")

            # 同时更新RViz悬浮窗口的速度显示
            if hasattr(self, 'speed_value') and self.speed_value:
                try:
                    speed_formatted = f"{self.linear_speed:.2f}"
                    unit = self.speed_value.property("unit") or ""
                    self.speed_value.setText(f"{speed_formatted} {unit}".strip())
                except Exception as e:
                    pass  # 静默处理错误

            # 标记速度话题有数据
            self.topics_with_data["velocity"] = True
        except Exception as e:
            print(f"更新速度显示时出错: {str(e)}")
    
    def updateStatusDisplay(self, status_data):
        """更新无人机状态信息显示"""
        try:
            if not status_data:
                return
                
            # 获取状态数据
            connected = status_data.get("connected", False)
            mode = status_data.get("mode", "")
            armed = status_data.get("armed", False)
            guided = status_data.get("guided", False)
            
            # 更新连接状态
            if hasattr(self, 'connection_label'):
                if connected:
                    self.connection_label.setText("已连接")
                    # 不设置字体大小，保持卡片的原始字体设置
                    self.connection_label.setStyleSheet("""
                        QLabel {
                            color: #2ECC71;
                            font-weight: bold;
                            background: transparent;
                            border: none;
                            padding: 0px;
                            margin: 0px;
                        }
                    """)
                else:
                    self.connection_label.setText("未连接")
                    # 不设置字体大小，保持卡片的原始字体设置
                    self.connection_label.setStyleSheet("""
                        QLabel {
                            color: #E74C3C;
                            font-weight: bold;
                            background: transparent;
                            border: none;
                            padding: 0px;
                            margin: 0px;
                        }
                    """)
            
            # 更新模式显示
            if hasattr(self, 'mode_label'):
                self.mode_label.setText(mode if mode else "UNKNOWN")

            # 同时更新RViz悬浮窗口的模式显示
            if hasattr(self, 'mode_value') and self.mode_value:
                try:
                    mode_text = mode if mode else "--"
                    self.mode_value.setText(mode_text)
                except Exception as e:
                    pass  # 静默处理错误

            # 标记话题有数据
            self.topics_with_data["status"] = True
            
        except Exception as e:
            print(f"更新状态显示时出错: {str(e)}")

    def updateRCDisplay(self, rc_data):
        """更新遥控器状态显示"""
        try:
            if not rc_data:
                # 没有数据时显示未连接
                if hasattr(self, 'rc_value') and self.rc_value:
                    self.rc_value.setText("未连接")
                    self.rc_value.setStyleSheet("color: #E74C3C; font-weight: bold;")
                return

            # 检查是否有通道数据
            channels = rc_data.get('channels', [])
            if channels and len(channels) > 0:
                # 有通道数据，显示已连接
                if hasattr(self, 'rc_value') and self.rc_value:
                    self.rc_value.setText("已连接")
                    self.rc_value.setStyleSheet("color: #2ECC71; font-weight: bold;")
            else:
                # 没有通道数据，显示未连接
                if hasattr(self, 'rc_value') and self.rc_value:
                    self.rc_value.setText("未连接")
                    self.rc_value.setStyleSheet("color: #E74C3C; font-weight: bold;")

            # 标记遥控器话题有数据
            self.topics_with_data["rc_input"] = True

        except Exception as e:
            pass  # 静默处理错误

    def updateCameraImage(self, camera_data):
        """处理摄像头图像更新 - 优化版本，确保实时更新"""
        try:
            if not camera_data or camera_data["image"] is None:
                return

            # 验证图像数据的有效性
            image = camera_data["image"]
            if not isinstance(image, np.ndarray):
                print("图像数据不是numpy数组")
                return

            if image.size == 0:
                print("图像数据为空")
                return

            # 检查图像维度
            if len(image.shape) not in [2, 3]:
                print(f"不支持的图像维度: {image.shape}")
                return

            # 保存最新图像（创建副本以确保数据安全）
            self.camera_image = image.copy()

            # 立即更新显示，无论当前模式如何
            # 这样确保当用户切换到RGB模式时能看到最新的图像
            if hasattr(self, 'image_label') and self.image_label:
                if self.current_image_mode == "rgb":
                    # 如果当前是RGB模式，立即更新显示
                    if pyqtSignal is not None and hasattr(self, 'image_update_signal'):
                        self.image_update_signal.emit()
                    else:
                        # 如果信号不可用，直接调用更新
                        self.updateImageDisplay()

        except Exception as e:
            print(f"处理图像更新时出错: {str(e)}")
            import traceback
            traceback.print_exc()

    def updateImageDisplay(self):
        """更新图像显示 - 优化版本，增强调试和错误处理"""
        try:
            if not hasattr(self, 'image_label') or not self.image_label:
                print("警告: image_label不存在或为None")
                return

            image_data = None

            # 根据当前模式选择对应的图像数据
            if self.current_image_mode == "rgb":
                if self.camera_image is not None:
                    image_data = self.camera_image
                    # print(f"使用RGB图像数据，尺寸: {self.camera_image.shape}")
                else:
                    # print("RGB模式但camera_image为None")
                    pass
            elif self.current_image_mode == "depth":
                if self.depth_image is not None:
                    image_data = self._process_depth_image(self.depth_image)
                    # print(f"使用深度图像数据，原始尺寸: {self.depth_image.shape}")
                else:
                    # print("深度模式但depth_image为None")
                    pass

            if image_data is not None:
                pixmap = self._convert_cv_to_pixmap(image_data)
                if pixmap and not pixmap.isNull():
                    success = self._scale_and_set_pixmap('image_label', pixmap)
                    if success:
                        # print(f"成功更新{self.current_image_mode}图像显示")
                        pass
                    else:
                        print("警告: 设置图像到标签失败")
                        self._show_image_placeholder()
                else:
                    print("警告: 图像转换为QPixmap失败")
                    self._show_image_placeholder()
            else:
                # print(f"没有可用的{self.current_image_mode}图像数据，显示占位符")
                self._show_image_placeholder()

        except Exception as e:
            print(f"更新图像显示时出错: {str(e)}")
            import traceback
            traceback.print_exc()
            if hasattr(self, 'image_label'):
                try:
                    self.image_label.setText(f"图像显示错误: {str(e)}")
                except Exception as e2:
                    print(f"设置错误文本时也出错: {str(e2)}")

    def _process_depth_image(self, depth_image):
        """处理深度图像"""
        cv_img = depth_image.copy()

        if len(cv_img.shape) == 2:  # 单通道深度图
            # 归一化到0-255，用于可视化
            min_val, max_val, _, _ = cv2.minMaxLoc(cv_img)
            if max_val > min_val:
                cv_img = cv2.convertScaleAbs(cv_img, alpha=255.0/(max_val-min_val), beta=-min_val*255.0/(max_val-min_val))
            # 应用彩色映射
            cv_img = cv2.applyColorMap(cv_img, cv2.COLORMAP_JET)

        return cv_img

    def _convert_cv_to_pixmap(self, cv_image):
        """将OpenCV图像转换为QPixmap - 安全版本"""
        try:
            if cv_image is None:
                print("输入图像为None")
                return None

            # 检查图像是否有效
            if cv_image.size == 0:
                print("图像数据为空")
                return None

            # 检查图像数据类型
            if not isinstance(cv_image, np.ndarray):
                print(f"图像数据类型错误: {type(cv_image)}")
                return None

            # 确保图像数据是连续的
            if not cv_image.flags['C_CONTIGUOUS']:
                cv_image = np.ascontiguousarray(cv_image)

            # 确保是3通道BGR图像
            if len(cv_image.shape) == 3:
                height, width, channel = cv_image.shape
                if channel != 3:
                    print(f"不支持的通道数: {channel}")
                    return None
                # 将BGR转换为RGB格式
                try:
                    rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                except Exception as e:
                    print(f"BGR到RGB转换失败: {e}")
                    return None
            elif len(cv_image.shape) == 2:
                # 单通道图像转换为RGB
                try:
                    rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
                    height, width, channel = rgb_image.shape
                except Exception as e:
                    print(f"灰度到RGB转换失败: {e}")
                    return None
            else:
                print(f"不支持的图像维度: {cv_image.shape}")
                return None

            # 确保转换后的图像数据是连续的
            if not rgb_image.flags['C_CONTIGUOUS']:
                rgb_image = np.ascontiguousarray(rgb_image)

            bytes_per_line = 3 * width

            # 创建QImage时使用copy确保数据安全
            try:
                q_image = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)

                if q_image.isNull():
                    print("创建QImage失败")
                    return None

                # 创建QPixmap的副本以确保数据生命周期
                pixmap = QPixmap.fromImage(q_image.copy())

                if pixmap.isNull():
                    print("创建QPixmap失败")
                    return None

                return pixmap
            except Exception as e:
                print(f"创建QImage/QPixmap时出错: {e}")
                return None

        except Exception as e:
            print(f"图像转换错误: {e}")
            import traceback
            traceback.print_exc()
            return None

    def _show_image_placeholder(self):
        """显示图像占位符"""
        if not self.topic_subscriber:
            text = "正在自动连接话题，请稍候..."
        elif self.current_image_mode == "rgb":
            if not self.topic_subscriber.is_topic_active("camera"):
                text = "等待RGB图像话题连接..."
            else:
                text = "等待RGB图像数据..."
        else:  # depth模式
            if not self.topic_subscriber.is_topic_active("depth"):
                text = "等待深度图像话题连接..."
            else:
                text = "等待深度图像数据..."

        # 使用更好的居中样式，确保文字在标签中完全居中
        self.image_label.setText(f"""
            <div style='
                display: flex;
                align-items: center;
                justify-content: center;
                height: 100%;
                width: 100%;
                font-size: 16pt;
                color: #3498DB;
                text-align: center;
                font-weight: bold;
            '>
                {text}
            </div>
        """)
    
    def updateStatusBar(self):
        # 更新帧率
        current_time = time.time()
        elapsed_time = current_time - self.last_frame_time
        if elapsed_time > 0:
            frame_rate = self.frame_count / elapsed_time
            self.frame_rate_label.setText(f"FPS: {frame_rate:.2f}")
            self.frame_count = 0
            self.last_frame_time = current_time
        
        # 更新ROS时间，精确到小数点后三位
        if not rospy.is_shutdown():
            try:
                # 检查ROS节点是否已初始化
                if rospy.get_name() != "/unnamed":
                    # 使用rospy.Time.now()获取当前ROS时间，而不是rospy.get_time()
                    now = rospy.Time.now()
                    ros_time = now.to_sec()  # 转换为秒
                    self.ros_time_label.setText(f"Time: {ros_time:.4f}")
                else:
                    # 如果ROS节点未初始化，显示系统时间
                    self.ros_time_label.setText(f"Time: {time.time():.4f}")
            except Exception as e:
                # 如果发生异常，显示系统时间
                print(f"获取ROS时间出错: {str(e)}")
                self.ros_time_label.setText(f"Time: {time.time():.4f}")
        
        # 显示话题连接状态
        self.updateTopicStatus()
        
        # 如果没有话题订阅器，显示待启动状态
        if not self.topic_subscriber:
            # 显示静态状态信息而不使用模拟数据
            if hasattr(self, 'battery_icon_label'):
                self.battery_icon_label.setPixmap(QPixmap(":/images/icons/battery_100.svg").scaled(24, 24, Qt.KeepAspectRatio, Qt.SmoothTransformation))
            
            if hasattr(self, 'voltage_label'):
                self.voltage_label.setText("-- V")
            
            if hasattr(self, 'position_label'):
                self.position_label.setText("Position: (等待话题连接)")
            
            if hasattr(self, 'altitude_label'):
                self.altitude_label.setText("-- m")
                
            if hasattr(self, 'ground_speed_label'):
                self.ground_speed_label.setText("-- m/s")

            # 当没有话题订阅器时，不使用模拟数据
            return
        
        # 如果话题订阅器存在但话题还未连接，显示等待连接状态而不是模拟数据
        if not (
            self.topic_subscriber.is_topic_active("battery") or 
            self.topic_subscriber.is_topic_active("odometry") or
            self.topic_subscriber.is_topic_active("velocity") or
            self.topic_subscriber.is_topic_active("status") or
            self.topic_subscriber.is_topic_active("camera") or
            self.topic_subscriber.is_topic_active("attitude")
        ):
            # 显示等待连接状态
            if hasattr(self, 'battery_icon_label'):
                self.battery_icon_label.setPixmap(QPixmap(":/images/icons/battery_100.svg").scaled(24, 24, Qt.KeepAspectRatio, Qt.SmoothTransformation))
            
            if hasattr(self, 'voltage_label'):
                self.voltage_label.setText("-- V")
            
            if hasattr(self, 'position_label'):
                self.position_label.setText("Position: (等待无人机连接)")
            
            if hasattr(self, 'altitude_label'):
                self.altitude_label.setText("-- m")
                
            if hasattr(self, 'ground_speed_label'):
                self.ground_speed_label.setText("-- m/s")
            
            # 当没有/mavros/state话题连接时，显示为未连接状态
            if hasattr(self, 'connection_label'):
                if not self.topic_subscriber or not self.topic_subscriber.is_topic_active("status"):
                    # 未连接状态
                    self.connection_label.setText("未连接")
                    # 不设置字体大小，保持卡片的原始字体设置
                    self.connection_label.setStyleSheet("""
                        QLabel {
                            color: #E74C3C;
                            font-weight: bold;
                            background: transparent;
                            border: none;
                            padding: 0px;
                            margin: 0px;
                        }
                    """)
                # 注意：如果话题已连接，则由updateStatusDisplay函数更新状态
            
            # 当没有/mavros/state话题连接时，显示为未知模式
            if hasattr(self, 'mode_label'):
                if not self.topic_subscriber or not self.topic_subscriber.is_topic_active("status"):
                    self.mode_label.setText("未连接")  # 显示未连接而非随机模式
            
            # 话题未连接时，不生成模拟图像，只更新UI显示消息
            
            # 更新RGB图像显示文本 - 使用自定义HTML样式显示
            if hasattr(self, 'image_label'):
                if not self.topic_subscriber.is_topic_active("camera"):
                    if self.current_image_mode == "rgb":  # 只在RGB模式下更新
                        self.image_label.setText("""
                            <div style='
                                display: flex;
                                align-items: center;
                                justify-content: center;
                                height: 100%;
                                width: 100%;
                                font-size: 16pt;
                                color: #3498DB;
                                text-align: center;
                                font-weight: bold;
                            '>
                                等待RGB图像话题连接...
                            </div>
                        """)
                    # 确保未使用模拟图像
                    self.camera_image = None

            # 更新深度图像显示文本 - 使用自定义HTML样式显示
            if hasattr(self, 'image_label'):
                if not self.topic_subscriber.is_topic_active("depth"):
                    if self.current_image_mode == "depth":  # 只在深度模式下更新
                        self.image_label.setText("""
                            <div style='
                                display: flex;
                                align-items: center;
                                justify-content: center;
                                height: 100%;
                                width: 100%;
                                font-size: 16pt;
                                color: #3498DB;
                                text-align: center;
                                font-weight: bold;
                            '>
                                等待深度图像话题连接...
                            </div>
                        """)
                    # 确保未使用模拟图像
                    self.depth_image = None
                
            # 更新鸟瞰图显示文本 - 使用自定义HTML样式显示
            if hasattr(self, 'bird_view_label'):
                if not self.topic_subscriber.is_topic_active("bird_view"):
                    self.bird_view_label.setText("<div style='font-size: 14pt; color: #3498DB; text-align: center; margin-top: 100px;'>等待鸟瞰图话题连接...</div>")
                    # 确保未使用模拟图像
                    self.bird_view_image = None
        
        # 每次渲染帧时增加计数器
        self.frame_count += 1

    def updateTopicStatus(self):
        """更新并显示话题状态"""
        if not hasattr(self, 'topic_subscriber') or not self.topic_subscriber:
            # 如果话题订阅器未初始化，显示等待启动信息
            if hasattr(self, 'position_label'):
                tooltip = "话题订阅器正在初始化\n正在自动尝试连接ROS话题，请稍候..."
                self.position_label.setToolTip(tooltip)
                
                # 添加或更新状态指示器
                if not hasattr(self, 'topic_status_indicator'):
                    # 创建状态指示器
                    self.topic_status_indicator = QLabel("ℹ️")
                    self.topic_status_indicator.setStyleSheet("color: #3498DB; font-weight: bold; padding-left: 5px; padding-right: 20px;")
                    self.topic_status_indicator.setToolTip(tooltip)
                    
                    # 将指示器添加到位置标签后面
                    if self.position_label.parent():
                        layout = self.position_label.parent().layout()
                        if layout:
                            layout.addWidget(self.topic_status_indicator)
                else:
                    self.topic_status_indicator.setText("ℹ️")
                    self.topic_status_indicator.setStyleSheet("color: #3498DB; font-weight: bold; padding-left: 5px; padding-right: 20px;")
                    self.topic_status_indicator.setToolTip(tooltip)
                    self.topic_status_indicator.show()
            return
            
        # 构建状态信息
        inactive_topics = []
        no_data_topics = []
        
        # 检查每个话题的状态
        for topic_name, is_active in self.topic_subscriber.topics_active.items():
            if not is_active:
                inactive_topics.append(topic_name)
            elif is_active and not self.topics_with_data.get(topic_name, False):
                no_data_topics.append(topic_name)
        
        # 如果有inactive或no_data话题，更新状态信息
        if inactive_topics or no_data_topics:
            status_text = ""
            
            if inactive_topics:
                status_text += f"未连接话题: {', '.join(inactive_topics)}"
            
            if no_data_topics:
                if status_text:
                    status_text += " | "
                status_text += f"无数据话题: {', '.join(no_data_topics)}"
            
            # 更新状态栏
            if hasattr(self, 'position_label'):
                tooltip = "部分话题未连接或没有数据\n\n"
                if inactive_topics:
                    tooltip += f"未连接话题:\n{', '.join(inactive_topics)}\n\n"
                if no_data_topics:
                    tooltip += f"已连接但无数据话题:\n{', '.join(no_data_topics)}"
                
                self.position_label.setToolTip(tooltip)
                
                # 在状态栏显示红点指示器
                if not hasattr(self, 'topic_status_indicator'):
                    # 创建状态指示器
                    self.topic_status_indicator = QLabel("⚠️")
                    self.topic_status_indicator.setStyleSheet("color: #E74C3C; font-weight: bold; padding-left: 5px; padding-right: 20px;")
                    self.topic_status_indicator.setToolTip(tooltip)
                    
                    # 将指示器添加到位置标签后面
                    if self.position_label.parent():
                        layout = self.position_label.parent().layout()
                        if layout:
                            layout.addWidget(self.topic_status_indicator)
                else:
                    self.topic_status_indicator.setText("⚠️")
                    self.topic_status_indicator.setStyleSheet("color: #E74C3C; font-weight: bold; padding-left: 5px; padding-right: 20px;")
                    self.topic_status_indicator.setToolTip(tooltip)
                    self.topic_status_indicator.show()
        elif hasattr(self, 'topic_status_indicator'):
            # 如果所有话题都正常，隐藏指示器
            self.topic_status_indicator.hide()

    def toggleRightSidebarPinned(self):
        """切换右侧栏的固定状态"""
        if self.right_sidebar_pinned:
            # 如果当前是固定状态，解除固定并隐藏
            self.right_sidebar_pinned = False
            self.toggleRightSidebar(hide=True, animate=True)
            # 更新按钮样式，恢复正常
            self.toggle_right_sidebar_btn.setStyleSheet("""
                QPushButton {
                    background-color: #1A202C;  /* 与周围颜色相协调 */
                    border: none;
                    border-radius: 0;
                    padding: 2px;
                }
                QPushButton:hover {
                    background-color: #3498DB;  /* 蓝色悬停效果 */
                }
                QPushButton:pressed {
                    background-color: #2980B9;  /* 按下效果 */
                }
            """)
        else:
            # 如果当前非固定，切换为固定状态并显示
            self.right_sidebar_pinned = True
            self.toggleRightSidebar(hide=False, animate=True)
            # 更新按钮样式，显示固定状态
            self.toggle_right_sidebar_btn.setStyleSheet("""
                QPushButton {
                    background-color: #3498DB;  /* 蓝色背景表示已固定 */
                    border: none;
                    border-radius: 0;
                    padding: 2px;
                }
                QPushButton:hover {
                    background-color: #2980B9;
                }
                QPushButton:pressed {
                    background-color: #2980B9;
                }
            """)

        # 延迟更新图像尺寸以适应侧边栏变化
        QTimer.singleShot(300, self.updateImageSizes)
        # 延迟更新表格列宽
        QTimer.singleShot(350, self.setupTableColumnWidths)
    
    def toggleRightSidebar(self, hide=None, animate=False):
        """显示或隐藏右侧栏
        
        参数:
            hide: 是否隐藏右侧栏。如果为None，则切换当前状态
            animate: 是否使用动画效果
        """
        # 如果指定了hide参数，则根据参数决定是否隐藏
        should_hide = hide if hide is not None else self.right_sidebar_expanded
        
        # 如果已经在动画中，则不重复触发
        if hasattr(self, 'right_sidebar_animation') and self.right_sidebar_animation.state() == QPropertyAnimation.Running:
            return

        # 清理之前的动画对象
        if hasattr(self, 'right_sidebar_animation'):
            try:
                self.right_sidebar_animation.finished.disconnect()
                self.right_sidebar_animation.valueChanged.disconnect()
                self.right_sidebar_animation.stop()
                self.right_sidebar_animation.deleteLater()
            except:
                pass  # 忽略断开连接时的错误

        if should_hide:
            # 隐藏右侧栏
            if animate:
                # 使用动画效果
                self.right_sidebar_animation = QPropertyAnimation(self.right_sidebar, b"maximumWidth")
                self.right_sidebar_animation.setDuration(200)  # 动画持续时间200ms
                current_width = self.right_sidebar.width()
                self.right_sidebar_animation.setStartValue(current_width)
                self.right_sidebar_animation.setEndValue(0)
                self.right_sidebar_animation.setEasingCurve(QEasingCurve.InOutQuad)

                # 确保右侧栏可见性正确
                self.right_sidebar.setVisible(True)

                # 动画结束后更新状态
                self.right_sidebar_animation.finished.connect(lambda: self.finishRightSidebarAnimation(False))

                # 动画过程中定期更新悬浮窗口位置
                self.right_sidebar_animation.valueChanged.connect(lambda: self._update_overlay_positions())

                # 启动动画
                self.right_sidebar_animation.start()
                
                # 立即更新状态，但不隐藏右侧栏（等动画完成）
                self.updateRightSidebarState(False)
                
                # 更新分割器尺寸
                sizes = self.main_splitter.sizes()
                new_sizes = [sizes[0], sizes[1], sizes[2] + sizes[4], sizes[3], 0]
                self.main_splitter.setSizes(new_sizes)
            else:
                # 直接隐藏
                self.right_sidebar.setMaximumWidth(0)
                self.right_sidebar.setMinimumWidth(0)
                self.right_sidebar.setVisible(False)
                self.updateRightSidebarState(False)
                
                # 更新分割器尺寸
                sizes = self.main_splitter.sizes()
                new_sizes = [sizes[0], sizes[1], sizes[2] + sizes[4], sizes[3], 0]
                self.main_splitter.setSizes(new_sizes)

                # 立即更新悬浮窗口位置
                QTimer.singleShot(50, self._update_overlay_positions)
        else:
            # 显示右侧栏
            if animate:
                # 清理之前的动画对象（如果存在）
                if hasattr(self, 'right_sidebar_animation'):
                    try:
                        self.right_sidebar_animation.finished.disconnect()
                        self.right_sidebar_animation.valueChanged.disconnect()
                        self.right_sidebar_animation.stop()
                        self.right_sidebar_animation.deleteLater()
                    except:
                        pass  # 忽略断开连接时的错误

                # 先设置最大宽度，以便动画可以工作
                self.right_sidebar.setMaximumWidth(self.adaptive_right_width)
                self.right_sidebar.setMinimumWidth(0)
                self.right_sidebar.setVisible(True)

                # 使用动画效果
                self.right_sidebar_animation = QPropertyAnimation(self.right_sidebar, b"maximumWidth")
                self.right_sidebar_animation.setDuration(200)  # 动画持续时间200ms
                self.right_sidebar_animation.setStartValue(0)
                self.right_sidebar_animation.setEndValue(self.adaptive_right_width)
                self.right_sidebar_animation.setEasingCurve(QEasingCurve.InOutQuad)

                # 动画结束后更新状态
                self.right_sidebar_animation.finished.connect(lambda: self.finishRightSidebarAnimation(True))

                # 动画过程中定期更新悬浮窗口位置
                self.right_sidebar_animation.valueChanged.connect(lambda: self._update_overlay_positions())

                # 启动动画
                self.right_sidebar_animation.start()

                # 立即更新状态
                self.updateRightSidebarState(True)

                # 更新分割器尺寸
                sizes = self.main_splitter.sizes()
                if sizes[2] > self.adaptive_right_width:  # 确保中间区域有足够空间
                    new_sizes = [sizes[0], sizes[1], sizes[2] - self.adaptive_right_width, sizes[3], self.adaptive_right_width]
                else:  # 如果中间区域空间不足，则按比例分配
                    total_space = sizes[2]
                    new_middle = max(int(total_space * 0.4), 100)  # 至少保留100px给中间区域
                    new_sizes = [sizes[0], sizes[1], new_middle, sizes[3], total_space - new_middle]
                self.main_splitter.setSizes(new_sizes)
            else:
                # 直接显示
                self.right_sidebar.setFixedWidth(self.adaptive_right_width)  # 使用自适应宽度
                self.right_sidebar.setVisible(True)
                self.updateRightSidebarState(True)

                # 更新分割器尺寸
                sizes = self.main_splitter.sizes()
                if sizes[2] > self.adaptive_right_width:  # 确保中间区域有足够空间
                    new_sizes = [sizes[0], sizes[1], sizes[2] - self.adaptive_right_width, sizes[3], self.adaptive_right_width]
                else:  # 如果中间区域空间不足，则按比例分配
                    total_space = sizes[2]
                    new_middle = max(int(total_space * 0.4), 100)  # 至少保留100px给中间区域
                    new_sizes = [sizes[0], sizes[1], new_middle, sizes[3], total_space - new_middle]
                self.main_splitter.setSizes(new_sizes)

                # 立即更新悬浮窗口位置
                QTimer.singleShot(50, self._update_overlay_positions)

        # 延迟更新图像尺寸以适应侧边栏变化
        QTimer.singleShot(250, self.updateImageSizes)
        # 延迟更新表格列宽
        QTimer.singleShot(300, self.setupTableColumnWidths)
                
    def finishRightSidebarAnimation(self, expanded):
        """右侧栏动画结束后的处理

        参数:
            expanded: 是否展开
        """
        if not expanded:
            # 动画结束后，如果是隐藏状态，则设置不可见以减少资源占用
            self.right_sidebar.setVisible(False)
        else:
            # 如果是显示状态，确保最小宽度也设置好
            self.right_sidebar.setMinimumWidth(self.adaptive_right_width)

        # 动画完成后立即更新悬浮窗口位置，使用多次延迟更新确保位置正确
        self._update_overlay_positions()
        QTimer.singleShot(50, self._update_overlay_positions)
        QTimer.singleShot(100, self._update_overlay_positions)
        QTimer.singleShot(200, self._update_overlay_positions)
    
    def updateRightSidebarState(self, expanded):
        """更新右侧栏状态

        参数:
            expanded: 是否展开
        """
        self.right_sidebar_expanded = expanded
        self.toggle_right_sidebar_btn.setIcon(QIcon(":/images/icons/dropright.svg" if expanded else ":/images/icons/dropleft.svg"))
        self.toggle_right_sidebar_btn.style().unpolish(self.toggle_right_sidebar_btn)
        self.toggle_right_sidebar_btn.style().polish(self.toggle_right_sidebar_btn)

    def toggleRVizDisplayPanel(self):
        """显示或隐藏RViz的原生显示面板"""
        try:
            # 使用RViz的setDisplayConfigVisible方法切换显示面板可见性
            display_visible = self.manager.getDisplayConfigVisibility()
            self.manager.setDisplayConfigVisibility(not display_visible)
            
            # 更新按钮文本
            if not display_visible:
                self.settings_button.setText("隐藏设置")
            else:
                self.settings_button.setText("设置")
        except Exception as e:
            print(f"切换RViz显示面板时出错: {str(e)}")

    def updateAttitudeDisplay(self, data=None):
        """更新姿态信息显示"""
        try:
            # 从话题数据中获取姿态信息
            if self.topic_subscriber and self.topic_subscriber.is_topic_active("attitude"):
                # 如果有真实姿态数据，使用真实数据
                attitude_data = data if data else self.topic_subscriber.get_latest_data("attitude")
                if attitude_data:
                    # 获取俯仰角并检查是否为列表
                    pitch_value = attitude_data.get("pitch", 0)
                    if isinstance(pitch_value, list):
                        if len(pitch_value) > 0:
                            pitch_value = pitch_value[0]
                        else:
                            pitch_value = 0
                    self.pitch = pitch_value
                    
                    # 获取滚转角并检查是否为列表
                    roll_value = attitude_data.get("roll", 0)
                    if isinstance(roll_value, list):
                        if len(roll_value) > 0:
                            roll_value = roll_value[0]
                        else:
                            roll_value = 0
                    self.roll = roll_value
                    
                    # 获取偏航角
                    yaw_value = attitude_data.get("yaw", 0)
                    if isinstance(yaw_value, list):
                        if len(yaw_value) > 0:
                            yaw_value = yaw_value[0]
                        else:
                            yaw_value = 0
                    
                    # 更新姿态标签 - 使用原始偏航角值
                    if hasattr(self, 'pitch_label'):
                        self.pitch_label.setText(f"{pitch_value:.2f}°")
                    if hasattr(self, 'roll_label'):
                        self.roll_label.setText(f"{roll_value:.2f}°")
                    if hasattr(self, 'yaw_label'):
                        self.yaw_label.setText(f"{-yaw_value:.2f}°")

                    # 同时更新RViz悬浮窗口组件
                    if hasattr(self, 'compass') and self.compass:
                        self.compass.set_heading(-yaw_value)

                    if hasattr(self, 'attitude_widget') and self.attitude_widget:
                        self.attitude_widget.update_attitude(pitch_value, roll_value)

                    # 保留对姿态指示器的更新，如果还在使用的话
                    if hasattr(self, 'attitude_indicator'):
                        self.attitude_indicator.update_attitude(self.pitch, self.roll)
            else:
                # 没有实际姿态数据时，显示默认值
                self.pitch = 0
                self.roll = 0
                if hasattr(self, 'pitch_label'):
                    self.pitch_label.setText("0.00°")
                if hasattr(self, 'roll_label'):
                    self.roll_label.setText("0.00°")
                if hasattr(self, 'yaw_label'):
                    self.yaw_label.setText("0.00°")
                
                # 如果姿态指示器还存在，也更新为零位
                if hasattr(self, 'attitude_indicator'):
                    self.attitude_indicator.update_attitude(self.pitch, self.roll)
        except Exception as e:
            print(f"更新姿态显示时出错: {str(e)}")
            
    def toggleLogWindow(self):
        """显示或隐藏日志窗口"""
        try:
            # 如果按钮被选中，但窗口不存在或已关闭
            if self.log_button.isChecked():
                # 如果窗口不存在，创建一个
                if not self.log_window or not hasattr(self.log_window, 'isVisible') or not self.log_window.isVisible():
                    if TopicLogger:
                        try:
                            from topic_logger import TopicLoggerDialog
                            self.log_window = TopicLoggerDialog(self)
                            # 窗口关闭时自动取消按钮选中状态
                            self.log_window.finished.connect(lambda: self.log_button.setChecked(False))
                            self.log_window.show()
                        except Exception as e:
                            print(f"创建日志窗口时出错: {str(e)}")
                            self.log_button.setChecked(False)
                    else:
                        print("话题日志组件不可用")
                        self.log_button.setChecked(False)
            else:
                # 如果按钮未选中，关闭窗口
                if self.log_window and hasattr(self.log_window, 'isVisible') and self.log_window.isVisible():
                    self.log_window.close()
        except Exception as e:
            print(f"切换日志窗口时出错: {str(e)}")
            self.log_button.setChecked(False)

    def startDroneSystem(self):
        """启动无人机系统"""
        try:
            # 创建日志目录 - 使用新的路径工具函数
            log_dir = get_data_directory("log")
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            
            # 显示正在启动的消息
            progress_dialog = QProgressDialog("正在启动无人机系统，请稍候...", "取消", 0, 100, self)
            progress_dialog.setWindowTitle("系统启动")
            progress_dialog.setWindowModality(Qt.WindowModal)
            progress_dialog.setCancelButton(None)  # 禁用取消按钮
            progress_dialog.setValue(0)
            progress_dialog.show()
            QApplication.processEvents()
            
            # 定义工作空间路径
            fast_drone_ws = os.path.expanduser("~/GUET_UAV_Drone_v2")
            zyc_fuel_ws = os.path.expanduser("~/zyc_fuel_ws")
            
            # 后台启动第一个程序 - 使用同步执行方式
            progress_dialog.setLabelText("正在启动主系统...")
            progress_dialog.setValue(10)
            QApplication.processEvents()
            
            # 创建日志文件
            main_system_log = f"{log_dir}/main_system_{timestamp}.log"
            self.log_files = {"main_system": main_system_log}
            print(f"主系统日志文件: {main_system_log}")
            
            cmd1 = f"cd {fast_drone_ws} && source {fast_drone_ws}/devel/setup.bash && sh shfiles/run.sh"
            with open(main_system_log, 'w') as log_file:
                process = subprocess.Popen(cmd1, shell=True, stdout=log_file, stderr=log_file, 
                                        executable='/bin/bash', text=True)
            
            # 等待25秒，确保所有节点启动完成
            timeout = 25  # 增加到25秒等待，与run.sh中的累计睡眠时间一致
            start_time = time.time()
            
            # 非阻塞方式检查进程是否已结束
            while time.time() - start_time < timeout:
                returncode = process.poll()
                if returncode is not None:  # 进程已结束
                    if returncode != 0:
                        # 获取错误输出
                        _, stderr = process.communicate()
                        error_msg = f"启动无人机系统失败，返回代码: {returncode}\n\n错误信息:\n{stderr[:500]}..."
                        QMessageBox.critical(self, "启动错误", error_msg)
                        progress_dialog.close()
                        return
                    break
                    
                # 更新进度条 - 在25秒内从10%逐步增加到50%
                elapsed = time.time() - start_time
                progress = int(10 + min(25, (elapsed / timeout * 25)))
                progress_dialog.setValue(progress)
                
                # 显示更有用的信息，包括剩余等待时间
                remaining = max(0, int(timeout - elapsed))
                progress_dialog.setLabelText(f"正在启动主系统...（还需等待约{remaining}秒）")
                
                QApplication.processEvents()
                time.sleep(0.5)  # 增加sleep间隔，减少UI更新频率
            
            # 无论脚本是否返回，都继续执行（run.sh是以后台方式运行各个节点的）
            print("已启动run.sh脚本，将等待其后台完成各节点启动")
            
            # 设置一个定时器检查进程是否在后续运行中出错
            self.check_process_timer = QTimer()
            self.check_process_timer.timeout.connect(lambda: self.checkProcessStatus(process, "主系统"))
            self.check_process_timer.start(5000)  # 每5秒检查一次
            
            # 继续执行，第一个脚本已经正常启动
            progress_dialog.setValue(50)
            progress_dialog.setLabelText("启动位姿转换模块...")
            QApplication.processEvents()
            
            # 延迟启动第二个进程 - 给run.sh额外的1秒时间完成启动
            QTimer.singleShot(1000, lambda: self.startSecondProcess(progress_dialog))
            
        except Exception as e:
            QMessageBox.critical(self, "启动错误", f"启动无人机系统时出错: {str(e)}")
    
    def monitorAllProcesses(self):
        """监视所有启动的进程状态"""
        try:
            if not hasattr(self, 'processes'):
                return
                
            for name, process in self.processes.items():
                if process is None:
                    continue
                    
                returncode = process.poll()
                if returncode is not None and returncode != 0:
                    # 进程已异常退出
                    try:
                        _, stderr = process.communicate(timeout=0.5)
                    except Exception:
                        stderr = "无法获取错误输出"
                        
                    error_msg = f"{name}进程异常终止，返回代码: {returncode}\n\n错误信息:\n{stderr[:500]}..."
                    QMessageBox.warning(self, "进程异常", error_msg)
                    
                    # 将进程标记为None，避免重复报警
                    self.processes[name] = None
        except Exception as e:
            print(f"监视进程时出错: {str(e)}")

    def checkProcessStatus(self, process, process_name):
        """检查进程状态，如果异常终止则显示错误"""
        try:
            # 检查该定时器是否已停止
            timer_name = f"check_process{process_name.split('系统')[0].strip() if '系统' in process_name else ''}_timer"
            timer_name = timer_name.replace("主", "")  # 处理"主系统"的特殊情况
            timer = getattr(self, timer_name, None)
            if timer is None or not timer.isActive():
                return False  # 定时器已停止，不再进行检查
                
            returncode = process.poll()
            if returncode is not None:  # 进程已结束
                # 先停止定时器，防止重复触发
                if timer:
                    timer.stop()
                
                # 只有在非正常退出且非SIGKILL/SIGTERM情况下才弹出错误
                # -9是SIGKILL, -15是SIGTERM, 这些通常是由停止按钮触发的，不应视为错误
                if returncode != 0 and returncode != -9 and returncode != -15:
                    # 获取错误输出
                    try:
                        _, stderr = process.communicate(timeout=0.5)  # 使用超时避免阻塞
                    except subprocess.TimeoutExpired:
                        stderr = "无法获取错误输出，进程可能仍在运行"
                    except Exception:
                        stderr = "无法获取错误输出"
                    
                    error_msg = f"{process_name}异常终止，返回代码: {returncode}\n\n错误信息:\n{stderr[:500]}..."
                    QMessageBox.critical(self, "运行错误", error_msg)
                    return False
            return True
        except Exception as e:
            print(f"检查进程状态时出错: {str(e)}")
            if hasattr(self, timer_name) and getattr(self, timer_name).isActive():
                getattr(self, timer_name).stop()  # 发生错误时也停止定时器
            return False
            
    def startSecondProcess(self, progress_dialog):
        """启动第二个进程"""
        try:
            # 定义工作空间路径
            zyc_fuel_ws = os.path.expanduser("~/zyc_fuel_ws")
            
            # 创建位姿转换模块日志文件
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            # 使用新的路径工具函数获取日志目录
            log_dir = get_data_directory("log")
            vins_log = f"{log_dir}/vins_to_mavros_{timestamp}.log"
            self.log_files["vins_to_mavros"] = vins_log
            print(f"位姿转换模块日志文件: {vins_log}")
            
            # 后台启动第二个程序
            cmd2 = f"cd {zyc_fuel_ws} && source {zyc_fuel_ws}/devel/setup.bash && rosrun vins_to_mavros vins_to_mavros_node"
            with open(vins_log, 'w') as log_file:
                process2 = subprocess.Popen(cmd2, shell=True, stdout=log_file, stderr=log_file, 
                                        executable='/bin/bash', text=True)
            
            # 等待2秒，检查初期启动情况
            timeout = 2  # 2秒超时检查
            start_time = time.time()
            
            # 非阻塞方式检查进程是否已结束
            while time.time() - start_time < timeout:
                returncode = process2.poll()
                if returncode is not None:  # 进程已结束
                    if returncode != 0:
                        # 获取错误输出
                        _, stderr = process2.communicate()
                        error_msg = f"启动位姿转换模块失败，返回代码: {returncode}\n\n错误信息:\n{stderr[:500]}..."
                        QMessageBox.critical(self, "启动错误", error_msg)
                        progress_dialog.close()
                        return
                    break
                    
                # 更新进度条
                progress = int(50 + min(25, (time.time() - start_time) / timeout * 25))
                progress_dialog.setValue(progress)
                QApplication.processEvents()
                time.sleep(0.1)
            
            # 设置一个定时器检查进程是否在后续运行中出错
            self.check_process2_timer = QTimer()
            self.check_process2_timer.timeout.connect(lambda: self.checkProcessStatus(process2, "位姿转换模块"))
            self.check_process2_timer.start(2000)  # 每2秒检查一次
            
            # 更新进度
            progress_dialog.setValue(75)
            progress_dialog.setLabelText("启动坐标转换模块...")
            QApplication.processEvents()
            
            # 延迟启动第三个进程
            QTimer.singleShot(3000, lambda: self.startThirdProcess(progress_dialog))
            
        except Exception as e:
            progress_dialog.close()
            QMessageBox.critical(self, "启动错误", f"启动第二个进程时出错: {str(e)}")
    
    def startThirdProcess(self, progress_dialog):
        """启动第三个进程"""
        try:
            # 定义工作空间路径
            zyc_fuel_ws = os.path.expanduser("~/zyc_fuel_ws")
            
            # 创建坐标转换模块日志文件
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            # 使用新的路径工具函数获取日志目录
            log_dir = get_data_directory("log")
            pose_to_odom_log = f"{log_dir}/pose_to_odom_{timestamp}.log"
            self.log_files["pose_to_odom"] = pose_to_odom_log
            print(f"坐标转换模块日志文件: {pose_to_odom_log}")
            
            # 后台启动第三个程序
            cmd3 = f"cd {zyc_fuel_ws} && source {zyc_fuel_ws}/devel/setup.bash && rosrun pose_to_odom_converter pose_to_odom_converter_node"
            with open(pose_to_odom_log, 'w') as log_file:
                process3 = subprocess.Popen(cmd3, shell=True, stdout=log_file, stderr=log_file, 
                                        executable='/bin/bash', text=True)
            
            # 等待2秒，检查初期启动情况
            timeout = 2  # 2秒超时检查
            start_time = time.time()
            
            # 非阻塞方式检查进程是否已结束
            while time.time() - start_time < timeout:
                returncode = process3.poll()
                if returncode is not None:  # 进程已结束
                    if returncode != 0:
                        # 获取错误输出
                        _, stderr = process3.communicate()
                        error_msg = f"启动坐标转换模块失败，返回代码: {returncode}\n\n错误信息:\n{stderr[:500]}..."
                        QMessageBox.critical(self, "启动错误", error_msg)
                        progress_dialog.close()
                        return
                    break
                    
                # 更新进度条
                progress = int(75 + min(20, (time.time() - start_time) / timeout * 20))
                progress_dialog.setValue(progress)
                QApplication.processEvents()
                time.sleep(0.1)
            
            # 设置一个定时器检查进程是否在后续运行中出错
            self.check_process3_timer = QTimer()
            self.check_process3_timer.timeout.connect(lambda: self.checkProcessStatus(process3, "坐标转换模块"))
            self.check_process3_timer.start(2000)  # 每2秒检查一次
            
            # 完成启动
            progress_dialog.setValue(100)
            QApplication.processEvents()
            progress_dialog.close()
            
            # 通知用户系统已成功启动基础模块，需要进一步校准
            QMessageBox.information(self, "初始启动完成", "无人机基础系统已启动，请准备进行摄像头位置校准！")
            
            # 自动打开日志窗口并显示odom话题的数据
            QTimer.singleShot(3000, self.showOdomLog)
            
        except Exception as e:
            progress_dialog.close()
            QMessageBox.critical(self, "启动错误", f"启动第三个进程时出错: {str(e)}")
    
    def setupTopicSubscriber(self):
        """初始化话题订阅器和相关回调函数"""
        try:
            # 如果已经有订阅器存在，先关闭它
            if self.topic_subscriber:
                self.topic_subscriber.shutdown()
                self.topic_subscriber = None
            
            # 重置话题数据状态标志
            self.topics_with_data = {
                "battery": False,
                "status": False,
                "odometry": False,
                "velocity": False,
                "camera": False,
                "depth": False,
                "bird_view": False,
                "marker": False
            }
            
            # 重置相关UI元素显示状态
            # 如果有电池状态显示，重置为初始状态
            if hasattr(self, 'battery_progress'):
                self.battery_progress.setValue(0)
                self.battery_percentage.setText("---%")
                self.battery_voltage.setText("--.- V")
            
            # 如果有位置显示，重置为初始状态
            if hasattr(self, 'position_value'):
                self.position_value.setText("x: ---m, y: ---m, z: ---m")
            
            # 如果有速度显示，重置为初始状态
            if hasattr(self, 'velocity_value'):
                self.velocity_value.setText("---m/s")
                
            # 如果有状态显示，重置为初始状态
            if hasattr(self, 'status_value'):
                self.status_value.setText("未连接")
                
            # 如果有相机图像显示，清空图像
            if hasattr(self, 'camera_image'):
                self.camera_image = None
                if hasattr(self, 'image_label'):
                    self.image_label.setText("""
                        <div style='
                            display: flex;
                            align-items: center;
                            justify-content: center;
                            height: 100%;
                            width: 100%;
                            font-size: 16pt;
                            color: #3498DB;
                            text-align: center;
                            font-weight: bold;
                        '>
                            等待图像数据...
                        </div>
                    """)
                
            # 创建新的订阅器
            self.topic_subscriber = TopicsSubscriber()
            
            # 注册回调函数
            self.topic_subscriber.register_callback("battery", self.updateBatteryStatus)
            self.topic_subscriber.register_callback("odometry", self.updatePositionDisplay)
            self.topic_subscriber.register_callback("velocity", self.updateVelocityDisplay)
            self.topic_subscriber.register_callback("status", self.updateStatusDisplay)
            self.topic_subscriber.register_callback("rc_input", self.updateRCDisplay)
            self.topic_subscriber.register_callback("camera", self.updateCameraImage)
            self.topic_subscriber.register_callback("depth", self.updateDepthImage)
            self.topic_subscriber.register_callback("bird_view", self.updateBirdViewImage)
            self.topic_subscriber.register_callback("marker", self.marker_callback)
            self.topic_subscriber.register_callback("attitude", self.updateAttitudeDisplay)
            
            # 注意：已移除MAVROS话题回调，使用普通话题替代
            
            print("话题订阅器已启动，将在后台自动连接可用话题...")
            return True
        except Exception as e:
            print(f"初始化话题订阅器失败: {str(e)}")
            self.topic_subscriber = None
            return False
            


    def showOdomLog(self):
        """显示odom话题的日志"""
        try:
            # 先确保日志窗口打开
            if not self.log_button.isChecked():
                self.log_button.click()
                
            # 等待日志窗口显示
            QTimer.singleShot(500, lambda: self.selectOdomTopic())
            
            # 显示摄像头校准对话框
            QTimer.singleShot(3000, self.showCameraCalibrationDialog)
            
        except Exception as e:
            print(f"显示odom话题日志时出错: {str(e)}")
    
    def showCameraCalibrationDialog(self):
        """显示摄像头校准对话框"""
        try:
            # 创建对话框
            dialog = QDialog(self)
            dialog.setWindowTitle("摄像头校准")
            dialog.setMinimumSize(400, 200)
            dialog.setStyleSheet("""
                QDialog {
                    background-color: #1E2330;
                    color: #FFFFFF;
                }
                QLabel {
                    color: #FFFFFF;
                    font-size: 14pt;
                }
                QPushButton {
                    background-color: #2980B9;
                    color: white;
                    border-radius: 4px;
                    padding: 10px 20px;
                    font-size: 12pt;
                    font-weight: bold;
                    min-height: 40px;
                }
                QPushButton#completeBtn {
                    background-color: #27AE60;
                }
                QPushButton#completeBtn:hover {
                    background-color: #2ECC71;
                }
                QPushButton#terminateBtn {
                    background-color: #E74C3C;
                }
                QPushButton#terminateBtn:hover {
                    background-color: #C0392B;
                }
            """)
            
            # 创建垂直布局
            layout = QVBoxLayout(dialog)
            
            # 添加提示标签
            label = QLabel("请对无人机摄像头位置进行校准")
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("margin-bottom: 20px;")
            layout.addWidget(label)
            
            # 添加按钮布局
            button_layout = QHBoxLayout()
            
            # 终止按钮
            terminate_btn = QPushButton("终止")
            terminate_btn.setObjectName("terminateBtn")
            terminate_btn.clicked.connect(lambda: self.handleCalibrationResponse(dialog, False))
            button_layout.addWidget(terminate_btn)
            
            # 校准完成按钮
            complete_btn = QPushButton("校准完成")
            complete_btn.setObjectName("completeBtn")
            complete_btn.clicked.connect(lambda: self.handleCalibrationResponse(dialog, True))
            button_layout.addWidget(complete_btn)
            
            # 添加按钮布局到主布局
            layout.addLayout(button_layout)
            
            # 显示对话框（模态）
            dialog.setModal(True)
            dialog.exec_()
            
        except Exception as e:
            print(f"显示摄像头校准对话框时出错: {str(e)}")
            # 如果对话框显示出错，继续执行额外脚本
            self.executeAdditionalScripts()
    
    def handleCalibrationResponse(self, dialog, completed):
        """处理校准对话框的响应"""
        try:
            # 关闭对话框
            dialog.accept()
            
            if completed:
                # 用户点击了"校准完成"，继续执行额外脚本
                QMessageBox.information(self, "校准完成", "准备启动无人机探索系统...")
                self.executeAdditionalScripts()
            else:
                # 用户点击了"终止"，停止后续执行
                QMessageBox.warning(self, "启动终止", "系统启动已被用户终止")
                
        except Exception as e:
            print(f"处理校准响应时出错: {str(e)}")
    
    def executeAdditionalScripts(self):
        """执行额外的启动脚本"""
        try:
            # 创建日志目录 - 使用新的路径工具函数
            log_dir = get_data_directory("log")
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            
            # 存储所有日志文件路径
            if not hasattr(self, 'log_files'):
                self.log_files = {}
            
            # 显示进度对话框
            progress_dialog = QProgressDialog("正在启动无人机探索系统...", "取消", 0, 100, self)
            progress_dialog.setWindowTitle("系统启动")
            progress_dialog.setWindowModality(Qt.WindowModal)
            progress_dialog.setCancelButton(None)  # 禁用取消按钮
            progress_dialog.setValue(0)
            progress_dialog.show()
            QApplication.processEvents()
            
            # 定义工作空间路径
            zyc_fuel_ws = os.path.expanduser("~/zyc_fuel_ws")
            shiyan_catkin_ws_target = os.path.expanduser("~/shiyan_catkin_ws_target")

            # 更新进度
            progress_dialog.setValue(10)
            progress_dialog.setLabelText("启动探索管理器...")
            QApplication.processEvents()

            # 创建探索管理器日志文件
            exploration_log = f"{log_dir}/exploration_manager_{timestamp}.log"
            self.log_files["exploration_manager"] = exploration_log
            print(f"探索管理器日志文件: {exploration_log}")

            # 启动探索管理器（后台运行，输出重定向到日志文件）
            cmd1 = f"cd {zyc_fuel_ws} && source devel/setup.bash && roslaunch exploration_manager exploration.launch"
            with open(exploration_log, 'w') as log_file:
                exploration_process = subprocess.Popen(cmd1, shell=True, stdout=log_file, stderr=log_file, executable='/bin/bash')
            
            # 等待探索管理器启动
            wait_time = 0
            max_wait = 10  # 最多等待10秒
            while wait_time < max_wait:
                # 更新等待信息
                progress_dialog.setLabelText(f"启动探索管理器...({wait_time+1}/{max_wait}秒)")
                QApplication.processEvents()
                
                # 检查探索管理器是否已启动
                try:
                    check_cmd = "rosnode list | grep -q exploration"
                    check_result = subprocess.run(check_cmd, shell=True)
                    if check_result.returncode == 0:
                        print("检测到探索管理器节点已启动")
                        break
                except Exception as e:
                    print(f"检查探索管理器节点时出错: {str(e)}")
                
                time.sleep(1)
                wait_time += 1
                progress_dialog.setValue(10 + int(wait_time * 20 / max_wait))  # 进度从10%逐渐增加到30%

            # 更新进度
            progress_dialog.setValue(30)
            progress_dialog.setLabelText("启动YOLO检测器...")
            QApplication.processEvents()
            
            # 创建YOLO检测器日志文件
            yolo_log = f"{log_dir}/yolo_detector_{timestamp}.log"
            self.log_files["yolo_detector"] = yolo_log
            print(f"YOLO检测器日志文件: {yolo_log}")
            
            # 启动YOLO检测器（后台运行，输出重定向到日志文件）
            cmd2 = f"cd {shiyan_catkin_ws_target} && source devel/setup.bash && roslaunch yolo_detector yolo_ros.launch"
            with open(yolo_log, 'w') as log_file:
                yolo_process = subprocess.Popen(cmd2, shell=True, stdout=log_file, stderr=log_file, executable='/bin/bash')
            
            # 等待并检查YOLO是否正常启动
            wait_time = 0
            max_wait = 20  # 最多等待20秒
            while wait_time < max_wait:
                # 每秒更新一次进度条和等待时间
                progress_dialog.setValue(30 + int(wait_time * 10 / max_wait))
                progress_dialog.setLabelText(f"启动YOLO检测器...({wait_time}/{max_wait}秒)")
                QApplication.processEvents()
                
                # 如果进程已结束且返回非零值，说明启动失败
                if yolo_process.poll() is not None and yolo_process.returncode != 0:
                    # 读取日志文件中的错误信息
                    with open(yolo_log, 'r') as log_file:
                        last_lines = log_file.readlines()[-20:] if log_file.readable() else []
                        error_msg = "启动YOLO检测器失败，错误信息:\n" + "".join(last_lines)
                    
                    print(f"YOLO检测器启动失败: {error_msg}")
                    QMessageBox.warning(self, "启动警告", "YOLO检测器可能启动失败，已记录到日志文件")
                    break
                
                # 尝试确认YOLO是否已启动
                try:
                    # 检查相关节点或话题是否存在
                    check_cmd = "rosnode list | grep -q yolo"
                    check_result = subprocess.run(check_cmd, shell=True)
                    if check_result.returncode == 0:
                        print("检测到YOLO节点已启动")
                        break
                except Exception as e:
                    print(f"检查YOLO节点时出错: {str(e)}")
                
                time.sleep(1)
                wait_time += 1

            # 更新进度
            progress_dialog.setValue(50)
            progress_dialog.setLabelText("启动SORT跟踪...")
            QApplication.processEvents()

            # 创建SORT跟踪日志文件
            sort_log = f"{log_dir}/sort_ros_{timestamp}.log"
            self.log_files["sort_ros"] = sort_log
            print(f"SORT跟踪日志文件: {sort_log}")

            # 启动SORT跟踪（后台运行，输出重定向到日志文件）
            cmd3 = f"cd {shiyan_catkin_ws_target} && source devel/setup.bash && roslaunch sort_ros sort_ros.launch"
            with open(sort_log, 'w') as log_file:
                sort_process = subprocess.Popen(cmd3, shell=True, stdout=log_file, stderr=log_file, executable='/bin/bash')
            
            # 等待SORT启动
            time.sleep(5)  # 等待5秒

            # 更新进度
            progress_dialog.setValue(70)
            progress_dialog.setLabelText("启动目标位置跟踪脚本...")
            QApplication.processEvents()
            time.sleep(3)  # 等待3秒

            # 创建小球位置跟踪脚本日志文件
            ball_tracker_log = f"{log_dir}/ball_pose_tracker_{timestamp}.log"
            self.log_files["ball_pose_tracker"] = ball_tracker_log
            print(f"小球位置跟踪脚本日志文件: {ball_tracker_log}")

            # 获取当前程序目录
            app_dir = get_application_directory()

            # 启动小球位置跟踪脚本（后台运行，输出重定向到日志文件）
            cmd4 = f"cd {app_dir} && python3 ball_pose_tracker.py"
            with open(ball_tracker_log, 'w') as log_file:
                ball_tracker_process = subprocess.Popen(cmd4, shell=True, stdout=log_file, stderr=log_file, executable='/bin/bash')

            # 将进程添加到进程管理字典中
            self.processes["ball_tracker"] = ball_tracker_process
            print(f"小球位置跟踪脚本已启动，PID: {ball_tracker_process.pid}")

            # 更新进度
            # progress_dialog.setValue(90)
            # progress_dialog.setLabelText("启动导航系统...")
            # QApplication.processEvents()
            # time.sleep(2)  # 等待2秒

            # # 创建导航系统日志文件
            # nav_log = f"{log_dir}/fuel_nav_{timestamp}.log"
            # self.log_files["fuel_nav"] = nav_log
            # print(f"导航系统日志文件: {nav_log}")

            # # 启动导航系统（后台运行，输出重定向到日志文件）
            # cmd5 = f"cd {zyc_fuel_ws} && source devel/setup.bash && rosrun exploration_manager fuel_nav"
            # with open(nav_log, 'w') as log_file:
            #     nav_process = subprocess.Popen(cmd5, shell=True, stdout=log_file, stderr=log_file, executable='/bin/bash')
            
            # 更新进度到100%
            progress_dialog.setValue(100)
            progress_dialog.setLabelText("完成所有启动步骤")
            QApplication.processEvents()
            time.sleep(1)  # 短暂延迟
            progress_dialog.close()
            

            # 话题订阅器已在程序启动时初始化，这里不需要再次调用
            
            # 显示成功消息和日志文件位置
            QMessageBox.information(self, "启动完成", f"无人机探索系统已启动!\n\n所有日志文件保存在:\n{log_dir}")
            
        except Exception as e:
            if 'progress_dialog' in locals() and progress_dialog is not None:
                progress_dialog.close()
            
            import traceback
            error_details = traceback.format_exc()
            print(f"执行额外脚本时出错:\n{error_details}")
            
            # 保存错误日志
            try:
                # 使用新的路径工具函数获取日志目录
                log_dir = get_data_directory("log")
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                error_log = f"{log_dir}/startup_error_{timestamp}.log"
                with open(error_log, 'w') as log_file:
                    log_file.write(error_details)
                QMessageBox.critical(self, "启动错误", f"执行额外脚本时出错: {str(e)}\n\n完整错误日志已保存到:\n{error_log}")
            except:
                QMessageBox.critical(self, "启动错误", f"执行额外脚本时出错: {str(e)}")
    
    def selectOdomTopic(self):
        """选择odom话题"""
        try:
            if self.log_window and hasattr(self.log_window, 'logger_widget'):
                # 查找/converted_odom话题
                odom_topic = "/converted_odom"
                combo = self.log_window.logger_widget.topic_combo
                
                # 寻找话题
                index = combo.findText(odom_topic)
                if index >= 0:
                    combo.setCurrentIndex(index)
                    # 点击打印按钮开始记录
                    self.log_window.logger_widget.log_btn.click()
                else:
                    print(f"找不到话题 {odom_topic}，等待话题可用")
                    # 再次尝试
                    QTimer.singleShot(2000, self.selectOdomTopic)
        except Exception as e:
            print(f"选择odom话题时出错: {str(e)}")

    def switchToRGBImage(self):
        """切换到RGB图像模式"""
        print("切换到RGB图像模式")
        self.rgb_button.setChecked(True)
        self.depth_button.setChecked(False)
        self.current_image_mode = "rgb"
        # 立即更新显示
        self.updateImageDisplay()

    def switchToDepthImage(self):
        """切换到深度图像模式"""
        print("切换到深度图像模式")
        self.rgb_button.setChecked(False)
        self.depth_button.setChecked(True)
        self.current_image_mode = "depth"
        # 立即更新显示
        self.updateImageDisplay()
    
    def updateDepthImage(self, depth_data):
        """处理深度图像更新 - 优化版本，确保实时更新"""
        try:
            if not depth_data or depth_data["image"] is None:
                return

            # 验证深度图像数据的有效性
            image = depth_data["image"]
            if not isinstance(image, np.ndarray):
                print("深度图像数据不是numpy数组")
                return

            if image.size == 0:
                print("深度图像数据为空")
                return

            # 保存最新深度图像（创建副本以确保数据安全）
            self.depth_image = image.copy()

            # 立即更新显示，无论当前模式如何
            # 这样确保当用户切换到深度模式时能看到最新的图像
            if hasattr(self, 'image_label') and self.image_label:
                if self.current_image_mode == "depth":
                    # 如果当前是深度模式，立即更新显示
                    if pyqtSignal is not None and hasattr(self, 'image_update_signal'):
                        self.image_update_signal.emit()
                    else:
                        # 如果信号不可用，直接调用更新
                        self.updateImageDisplay()

        except Exception as e:
            print(f"处理深度图像更新时出错: {str(e)}")
            import traceback
            traceback.print_exc()

    def updateBirdViewImage(self, bird_view_data):
        """处理鸟瞰图更新"""
        try:
            if not bird_view_data or bird_view_data["image"] is None:
                return
                
            # 保存最新鸟瞰图像
            self.bird_view_image = bird_view_data["image"]

            # 使用信号安全地更新鸟瞰图显示
            if pyqtSignal is not None and hasattr(self, 'bird_view_update_signal'):
                self.bird_view_update_signal.emit()
            else:
                # 如果信号不可用，直接调用（可能不安全，但保持兼容性）
                self.updateBirdViewDisplay()
                
        except Exception as e:
            import traceback
            print(f"处理鸟瞰图更新时出错: {str(e)}")
            print(traceback.format_exc())
    
    def updateBirdViewDisplay(self):
        """更新鸟瞰图显示 - 优化版本"""
        try:
            if not hasattr(self, 'bird_view_label') or not self.bird_view_label:
                return

            if self.bird_view_image is not None and self.bird_view_image.size > 0:
                pixmap = self._convert_cv_to_pixmap(self.bird_view_image)
                if pixmap:
                    self._scale_and_set_pixmap('bird_view_label', pixmap,
                                             self.adaptive_image_width,
                                             self.adaptive_bird_height)
                else:
                    self._show_bird_view_placeholder("图像转换失败")
            else:
                self._show_bird_view_placeholder("等待鸟瞰图数据...")

        except Exception as e:
            print(f"更新鸟瞰图显示时出错: {str(e)}")
            self._show_bird_view_placeholder(f"显示错误: {str(e)}")

    def _show_bird_view_placeholder(self, message):
        """显示鸟瞰图占位符"""
        if hasattr(self, 'bird_view_label'):
            self.bird_view_label.setText(f"<div style='font-size: 12pt; color: #3498DB; text-align: center; margin-top: 50px;'>{message}</div>")
    
    # 添加人员位置管理功能
    def addPerson(self):
        """添加搜索人员位置"""
        # 创建对话框
        dialog = QDialog(self)
        dialog.setWindowTitle("添加待搜索人员")
        dialog.setFixedWidth(300)
        dialog.setStyleSheet("""
            QDialog {
                background-color: #1E2330;
                color: #FFFFFF;
            }
            QLabel {
                color: #FFFFFF;
                font-size: 12px;
            }
            QLineEdit {
                padding: 5px;
                border: 1px solid #3498DB;
                border-radius: 3px;
                background-color: #2C3E50;
                color: white;
                selection-background-color: #3498DB;
            }
            QPushButton {
                background-color: #2980B9;
                color: white;
                border-radius: 4px;
                padding: 5px 15px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #3498DB;
            }
            QComboBox {
                padding: 5px;
                border: 1px solid #3498DB;
                border-radius: 3px;
                background-color: #2C3E50;
                color: white;
            }
            QComboBox::drop-down {
                border: 0px;
            }
            QComboBox::down-arrow {
                image: url(:/images/icons/dropdown.svg);
                width: 12px;
                height: 12px;
            }
            QComboBox QAbstractItemView {
                background-color: #2C3E50;
                color: white;
                selection-background-color: #3498DB;
                border: 1px solid #3498DB;
            }
        """)
        
        # 创建对话框布局
        layout = QVBoxLayout(dialog)
        
        # 添加表单字段
        form_layout = QFormLayout()
        
        # ID字段（自动生成）
        next_id = self.position_table.rowCount() + 1
        id_label = QLabel(f"ID: {next_id}")
        form_layout.addRow("", id_label)
        
        # X坐标字段
        x_edit = QLineEdit()
        x_edit.setValidator(QDoubleValidator())  # 接受浮点数
        form_layout.addRow("X坐标:", x_edit)
        
        # Y坐标字段
        y_edit = QLineEdit()
        y_edit.setValidator(QDoubleValidator())  # 接受浮点数
        form_layout.addRow("Y坐标:", y_edit)
        
        # 状态字段
        status_combo = QComboBox()
        status_combo.addItems(["待确认", "已确认", "已救援"])
        form_layout.addRow("状态:", status_combo)
        
        layout.addLayout(form_layout)
        
        # 添加按钮区
        button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        button_box.accepted.connect(dialog.accept)
        button_box.rejected.connect(dialog.reject)
        layout.addWidget(button_box)
        
        # 执行对话框
        if dialog.exec_() == QDialog.Accepted:
            try:
                # 获取输入值
                x_value = x_edit.text()
                y_value = y_edit.text()
                status = status_combo.currentText()
                
                # 验证输入
                if not x_value or not y_value:
                    QMessageBox.warning(self, "输入错误", "X坐标和Y坐标不能为空")
                    return
                    
                # 添加到表格
                row_position = self.position_table.rowCount()
                self.position_table.insertRow(row_position)
                
                # 设置单元格值
                self.position_table.setItem(row_position, 0, QTableWidgetItem(str(next_id)))
                self.position_table.setItem(row_position, 1, QTableWidgetItem(x_value))
                self.position_table.setItem(row_position, 2, QTableWidgetItem(y_value))
                status_item = QTableWidgetItem(status)
                
                # 设置状态颜色
                if status == "待确认":
                    status_item.setForeground(QBrush(QColor("#F39C12")))  # 橙色
                elif status == "已确认":
                    status_item.setForeground(QBrush(QColor("#2ECC71")))  # 绿色
                elif status == "已救援":
                    status_item.setForeground(QBrush(QColor("#3498DB")))  # 蓝色
                    
                self.position_table.setItem(row_position, 3, status_item)
                
                print(f"已添加新的搜索人员: ID={next_id}, X={x_value}, Y={y_value}, 状态={status}")
            except Exception as e:
                print(f"添加人员时出错: {str(e)}")
                QMessageBox.critical(self, "错误", f"添加人员时出错: {str(e)}")
    
    def removePerson(self):
        """删除选中的搜索人员"""
        # 获取选中的行
        selected_rows = set()
        for item in self.position_table.selectedItems():
            selected_rows.add(item.row())
        
        if not selected_rows:
            QMessageBox.warning(self, "提示", "请先选择要删除的人员")
            return
        
        # 确认是否删除
        confirm = QMessageBox.question(self, "确认删除", 
                                     f"确定要删除选中的{len(selected_rows)}个人员吗？", 
                                     QMessageBox.Yes | QMessageBox.No)
        
        if confirm == QMessageBox.Yes:
            # 从后向前删除行(避免索引变化)
            for row in sorted(selected_rows, reverse=True):
                person_id = self.position_table.item(row, 0).text()
                self.position_table.removeRow(row)
                print(f"已删除ID为{person_id}的人员记录")
    
    def updatePersonStatus(self):
        """更新选中人员的状态"""
        # 获取选中的行
        selected_items = self.position_table.selectedItems()
        
        if not selected_items:
            QMessageBox.warning(self, "提示", "请先选择要更新的人员")
            return
            
        # 获取唯一的行
        selected_rows = set()
        for item in selected_items:
            selected_rows.add(item.row())
        
        if len(selected_rows) > 1:
            # 创建状态选择对话框
            dialog = QDialog(self)
            dialog.setWindowTitle("批量更新状态")
            dialog.setFixedWidth(250)
            dialog.setStyleSheet("""
                QDialog {
                    background-color: #1E2330;
                    color: #FFFFFF;
                }
                QLabel {
                    color: #FFFFFF;
                    font-size: 12px;
                }
                QPushButton {
                    background-color: #2980B9;
                    color: white;
                    border-radius: 4px;
                    padding: 5px 15px;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background-color: #3498DB;
                }
                QComboBox {
                    padding: 5px;
                    border: 1px solid #3498DB;
                    border-radius: 3px;
                    background-color: #2C3E50;
                    color: white;
                }
            """)
            
            layout = QVBoxLayout(dialog)
            
            # 状态选择
            layout.addWidget(QLabel(f"为{len(selected_rows)}个选中人员设置新状态:"))
            status_combo = QComboBox()
            status_combo.addItems(["待确认", "已确认", "已救援"])
            layout.addWidget(status_combo)
            
            # 按钮
            button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
            button_box.accepted.connect(dialog.accept)
            button_box.rejected.connect(dialog.reject)
            layout.addWidget(button_box)
            
            # 执行对话框
            if dialog.exec_() == QDialog.Accepted:
                new_status = status_combo.currentText()
                # 更新所有选中行的状态
                for row in selected_rows:
                    self._updateRowStatus(row, new_status)
        else:
            # 单行更新，直接循环状态
            row = list(selected_rows)[0]
            current_status = self.position_table.item(row, 3).text()
            
            # 状态循环: 待确认 -> 已确认 -> 已救援 -> 待确认
            if current_status == "待确认":
                new_status = "已确认"
            elif current_status == "已确认":
                new_status = "已救援"
            else:
                new_status = "待确认"
            
            self._updateRowStatus(row, new_status)
    
    def _updateRowStatus(self, row, new_status):
        """更新指定行的状态"""
        person_id = self.position_table.item(row, 0).text()
        status_item = QTableWidgetItem(new_status)
        
        # 设置状态颜色
        if new_status == "待确认":
            status_item.setForeground(QBrush(QColor("#F39C12")))  # 橙色
        elif new_status == "已确认":
            status_item.setForeground(QBrush(QColor("#2ECC71")))  # 绿色
        elif new_status == "已救援":
            status_item.setForeground(QBrush(QColor("#3498DB")))  # 蓝色
            
        self.position_table.setItem(row, 3, status_item)
        print(f"已将ID为{person_id}的人员状态更新为{new_status}")
    
    def onResize(self, event):
        """窗口大小变化时调整组件尺寸"""
        try:
            # 获取当前窗口大小
            window_height = self.height()
            window_width = self.width()

            # 只有在窗口大小显著变化时才重新计算（避免频繁调整）
            if not hasattr(self, '_last_window_size'):
                self._last_window_size = (window_width, window_height)
                should_recalculate = True
            else:
                last_width, last_height = self._last_window_size
                width_diff = abs(window_width - last_width)
                height_diff = abs(window_height - last_height)
                # 只有当宽度或高度变化超过50px时才重新计算
                should_recalculate = width_diff > 50 or height_diff > 50

            if should_recalculate:
                # 更新记录的窗口大小
                self._last_window_size = (window_width, window_height)

                # 重新计算自适应尺寸
                old_screen_width = self.screen_width
                old_screen_height = self.screen_height
                self.screen_width = window_width
                self.screen_height = window_height
                self.calculateAdaptiveSizes()

                # 只有在尺寸真正改变时才更新组件
                if (old_screen_width != self.screen_width or old_screen_height != self.screen_height):
                    # 动态调整侧边栏宽度
                    if hasattr(self, 'left_sidebar'):
                        self.left_sidebar.setFixedWidth(self.adaptive_left_width)

                    if hasattr(self, 'right_sidebar'):
                        self.right_sidebar.setFixedWidth(self.adaptive_right_width)

                    # 重新设置分割器尺寸
                    if hasattr(self, 'main_splitter'):
                        # 使用定时器延迟调整，避免与侧边栏动画冲突
                        QTimer.singleShot(100, self._setAdaptiveSplitterSizes)

                    # 延迟更新图像尺寸，确保分割器调整完成后再更新
                    QTimer.singleShot(200, self.updateImageSizes)
                    # 延迟更新表格列宽
                    QTimer.singleShot(250, self.setupTableColumnWidths)
                    # 延迟更新悬浮窗口位置
                    QTimer.singleShot(300, self._update_overlay_positions)

            # 小窗口模式下简化功能组标题，避免截断
            if hasattr(self, 'function_group'):
                if window_width < 1600:
                    self.function_group.setTitle("🎮 控制中心")
                else:
                    self.function_group.setTitle("🎮 控制中心")

            # 调用原始的resizeEvent
            QMainWindow.resizeEvent(self, event)
        except Exception as e:
            print(f"调整窗口大小时出错: {str(e)}")
            # 确保原始事件被处理
            QMainWindow.resizeEvent(self, event)
    


    def marker_callback(self, marker_data):
        """处理visualization_marker话题的回调函数 - 安全版本"""
        try:
            # 验证输入数据
            if not marker_data or not isinstance(marker_data, dict):
                print("标记数据无效或为空")
                return

            # 安全地提取标记ID
            if "id" not in marker_data:
                print("标记数据中缺少ID字段")
                return

            marker_id = marker_data["id"]
            if not isinstance(marker_id, int):
                print(f"标记ID不是整数: {marker_id}")
                return

            if marker_id % 2 == 0:  # 球体标记的ID是偶数
                ball_id = marker_id // 2  # 获取实际的球体ID

                # 检查是否已添加过该标记
                if ball_id not in self.detected_markers:
                    # 安全地获取小球坐标
                    try:
                        x = marker_data["pose"]["position"]["x"]
                        y = marker_data["pose"]["position"]["y"]
                        z = marker_data["pose"]["position"]["z"]

                        # 验证坐标是否为有效数值
                        if not all(isinstance(coord, (int, float)) for coord in [x, y, z]):
                            print(f"坐标数据无效: x={x}, y={y}, z={z}")
                            return

                    except (KeyError, TypeError) as e:
                        print(f"提取坐标数据时出错: {str(e)}")
                        return

                    # 添加到表格中
                    self._add_marker_to_table(ball_id, x, y, z)

                    # 标记为已添加
                    self.detected_markers.add(ball_id)

                    print(f"检测到新的标记点: ID={ball_id}, 位置: x={x:.2f}, y={y:.2f}, z={z:.2f}")

        except Exception as e:
            print(f"处理标记点数据时出错: {str(e)}")
            import traceback
            traceback.print_exc()

    def _add_marker_to_table(self, ball_id, x, y, z):
        """将检测到的标记点添加到人员位置表格"""
        try:
            # 获取表格当前行数
            row_position = self.position_table.rowCount()
            self.position_table.insertRow(row_position)
            
            # 设置单元格值
            self.position_table.setItem(row_position, 0, QTableWidgetItem(str(ball_id)))
            self.position_table.setItem(row_position, 1, QTableWidgetItem(f"{x:.2f} m"))
            self.position_table.setItem(row_position, 2, QTableWidgetItem(f"{y:.2f} m"))
            
            # 设置状态为"待确认"
            status_item = QTableWidgetItem("待确认")
            status_item.setForeground(QBrush(QColor("#F39C12")))  # 橙色
            self.position_table.setItem(row_position, 3, status_item)
            
            # 查找预先保存的截图
            screenshot_status = self.load_ball_screenshot(ball_id)
            screenshot_item = QTableWidgetItem(screenshot_status)
            if screenshot_status == "查看截图":
                screenshot_item.setForeground(QBrush(QColor("#27AE60")))  # 绿色表示有截图
            else:
                screenshot_item.setForeground(QBrush(QColor("#E74C3C")))  # 红色表示无截图
            self.position_table.setItem(row_position, 4, screenshot_item)
            
            print(f"已添加标记点到表格: ID={ball_id}, X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
            
            # 自动滚动到新添加的行
            self.position_table.scrollToItem(self.position_table.item(row_position, 0))
        except Exception as e:
            print(f"添加标记点到表格时出错: {str(e)}")
            import traceback
            traceback.print_exc()

            # 如果添加行失败，尝试清理
            try:
                if hasattr(self, 'position_table') and self.position_table:
                    current_rows = self.position_table.rowCount()
                    if current_rows > 0:
                        # 检查最后一行是否为空，如果是则删除
                        last_row = current_rows - 1
                        if self.position_table.item(last_row, 0) is None:
                            self.position_table.removeRow(last_row)
            except:
                pass  # 忽略清理时的错误
            
    def load_ball_screenshot(self, ball_id):
        """加载小球的预先保存的截图"""
        try:
            # 检查ball_screenshots目录中是否有对应的截图文件
            ball_screenshots_dir = get_data_directory("ball_screenshots")

            if not os.path.exists(ball_screenshots_dir):
                print(f"截图目录不存在: {ball_screenshots_dir}")
                return "无截图"

            # 查找以ball_{ball_id}_开头的文件
            import glob
            pattern = os.path.join(ball_screenshots_dir, f"ball_{ball_id}_*.jpg")
            matching_files = glob.glob(pattern)

            if matching_files:
                # 如果找到多个文件，选择最新的
                latest_file = max(matching_files, key=os.path.getctime)

                # 验证文件是否可读
                if os.path.exists(latest_file) and os.path.getsize(latest_file) > 0:
                    # 存储截图信息
                    self.ball_screenshots[ball_id] = {
                        "path": latest_file,
                        "timestamp": os.path.getctime(latest_file)
                    }
                    print(f"找到小球 {ball_id} 的截图: {latest_file}")
                    return "查看截图"
                else:
                    print(f"截图文件损坏或为空: {latest_file}")
                    return "截图损坏"
            else:
                print(f"未找到小球 {ball_id} 的截图文件")
                return "无截图"

        except Exception as e:
            print(f"加载小球截图时出错: {str(e)}")
            return "加载失败"

    def capture_ball_screenshot(self, ball_id):
        """拍摄小球截图并保存（已弃用，现在使用预先保存的截图）"""
        # 这个函数现在不再使用，因为截图是在检测过程中预先保存的
        print(f"注意: capture_ball_screenshot 已弃用，小球 {ball_id} 的截图应该已经预先保存")

    def stopDroneSystem(self):
        """停止无人机系统"""
        try:
            # 确认弹窗
            reply = QMessageBox.question(self, "确认停止", "确定要停止所有无人机系统进程吗？",
                                      QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            
            if reply == QMessageBox.No:
                return
                
            # 立即停止所有进程监控定时器，避免重复弹出错误消息
            for timer_attr in ['check_process_timer', 'check_process2_timer', 'check_process3_timer', 'process_monitor_timer']:
                if hasattr(self, timer_attr):
                    timer = getattr(self, timer_attr)
                    if timer and timer.isActive():
                        timer.stop()
                        print(f"已停止{timer_attr}")
            
            # 显示进度对话框
            progress_dialog = QProgressDialog("正在停止无人机系统...", "取消", 0, 100, self)
            progress_dialog.setWindowTitle("系统停止")
            progress_dialog.setWindowModality(Qt.WindowModal)
            progress_dialog.setCancelButton(None)  # 禁用取消按钮
            progress_dialog.setValue(10)
            progress_dialog.show()
            QApplication.processEvents()  # 确保对话框显示出来
            
            # 如果有任何话题订阅器在运行，先关闭它
            if self.topic_subscriber:
                progress_dialog.setLabelText("正在关闭话题订阅...")
                progress_dialog.setValue(20)
                QApplication.processEvents()
                
                try:
                    self.topic_subscriber.shutdown()
                    self.topic_subscriber = None
                    print("已关闭话题订阅器")
                except Exception as e:
                    print(f"关闭话题订阅器时出错: {str(e)}")
                
                progress_dialog.setValue(30)
                QApplication.processEvents()
            
            # 逐个关闭之前启动的进程
            if hasattr(self, 'processes') and self.processes:
                total_processes = len([p for p in self.processes.values() if p is not None])
                progress_per_process = 50 / max(total_processes, 1)  # 在30%-80%的进度范围内分配
                current_progress = 30
                
                # 按照启动的相反顺序终止进程
                process_order = ["ball_tracker", "planner", "px4ctrl", "detector", "vins", "mavros", "camera"]
                
                for process_name in process_order:
                    if process_name in self.processes and self.processes[process_name]:
                        progress_dialog.setLabelText(f"正在停止{process_name}进程...")
                        progress_dialog.setValue(int(current_progress))
                        QApplication.processEvents()
                        
                        try:
                            # 终止进程
                            self.processes[process_name].terminate()
                            # 给进程一点时间自行退出
                            start_time = time.time()
                            while time.time() - start_time < 2:  # 最多等待2秒
                                if self.processes[process_name].poll() is not None:
                                    # 进程已结束
                                    break
                                time.sleep(0.1)
                            
                            # 如果进程仍未退出，强制杀死
                            if self.processes[process_name].poll() is None:
                                self.processes[process_name].kill()
                            
                            # 等待进程完全退出
                            self.processes[process_name].wait(timeout=1)
                            print(f"已停止{process_name}进程")
                        except Exception as e:
                            print(f"停止{process_name}进程时出错: {str(e)}")
                        
                        current_progress += progress_per_process
                
                # 清空进程列表
                self.processes = {}
                
                progress_dialog.setValue(60)
                QApplication.processEvents()
            
            # 使用强大的终止机制，确保所有相关进程都被终止
            progress_dialog.setLabelText("正在终止所有相关进程...")
            progress_dialog.setValue(70)
            QApplication.processEvents()
            
            # 使用全局常量避免重复定义
            process_patterns = PROCESS_PATTERNS
            
            # 使用pkill强制终止每个模式的进程
            for i, pattern in enumerate(process_patterns):
                progress_value = 70 + (i * 10 // len(process_patterns))
                progress_dialog.setValue(progress_value)
                progress_dialog.setLabelText(f"正在终止进程: {pattern}...")
                QApplication.processEvents()
                
                try:
                    # 使用pgrep检查进程是否存在
                    check_process = subprocess.run(
                        f"pgrep -f \"{pattern}\"", 
                        shell=True, 
                        stdout=subprocess.PIPE, 
                        stderr=subprocess.PIPE
                    )
                    
                    if check_process.returncode == 0:  # 进程存在
                        # 使用pkill -9 强制终止
                        kill_cmd = f"pkill -9 -f \"{pattern}\""
                        subprocess.run(kill_cmd, shell=True)
                        print(f"已终止进程：{pattern}")
                    else:
                        print(f"未找到进程：{pattern}")
                except Exception as e:
                    print(f"终止进程 {pattern} 时出错: {str(e)}")
            
            progress_dialog.setValue(80)
            progress_dialog.setLabelText("正在清理ROS节点...")
            QApplication.processEvents()
            
            # 保留roscore，但清理其他所有ROS节点
            try:
                # 先检查ROS环境是否正常
                rosnode_check = subprocess.run(
                    "rosnode list", 
                    shell=True, 
                    stdout=subprocess.PIPE, 
                    stderr=subprocess.PIPE, 
                    text=True, 
                    timeout=5
                )
                
                if rosnode_check.returncode == 0:
                    # 获取节点列表
                    nodes = rosnode_check.stdout.strip().split('\n')
                    # 过滤掉rosout和master相关节点
                    nodes_to_kill = [node for node in nodes if not any(x in node for x in ['/rosout', '/master'])]
                    
                    if nodes_to_kill:
                        # 每个节点单独终止
                        for i, node in enumerate(nodes_to_kill):
                            progress_value = 80 + (i * 10 // max(len(nodes_to_kill), 1))
                            progress_dialog.setValue(progress_value)
                            progress_dialog.setLabelText(f"正在清理节点: {node}...")
                            QApplication.processEvents()
                            
                            kill_cmd = f"rosnode kill {node}"
                            subprocess.run(kill_cmd, shell=True, timeout=2)
                            print(f"已清理节点: {node}")
                    
                    print("已清理所有非核心ROS节点")
                else:
                    print("ROS环境可能未启动或异常")
            except Exception as e:
                print(f"清理ROS节点时出错: {str(e)}")
                
            progress_dialog.setValue(90)
            QApplication.processEvents()
            
            # 创建一个简单的实时输出过程对象以保持兼容性
            class DummyProcess:
                def __init__(self):
                    self.returncode = 0
                    
            process = DummyProcess()
            
            # 准备收集输出
            all_output = []
            all_errors = []
            
            # 创建一个非阻塞读取输出的函数
            def read_output():
                # 使用非阻塞方式从stdout读取
                import select
                
                # 设置文件描述符为非阻塞模式
                import fcntl
                import os
                
                # 获取文件描述符
                stdout_fd = process.stdout.fileno()
                stderr_fd = process.stderr.fileno()
                
                # 设置非阻塞标志
                fl = fcntl.fcntl(stdout_fd, fcntl.F_GETFL)
                fcntl.fcntl(stdout_fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
                
                fl = fcntl.fcntl(stderr_fd, fcntl.F_GETFL)
                fcntl.fcntl(stderr_fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
                
                has_data = False
                
                # 使用select检查是否有数据可读，超时0.1秒
                readable, _, _ = select.select([stdout_fd, stderr_fd], [], [], 0.1)
                
                # 读取stdout
                if stdout_fd in readable:
                    try:
                        # 读一行或者最多4096字节
                        stdout_data = process.stdout.readline()
                        if stdout_data:
                            all_output.append(stdout_data)
                            # 更新进度对话框状态
                            cur_progress = min(40 + len(all_output) * 2, 90)  # 最大到90%
                            progress_dialog.setValue(cur_progress)
                            progress_dialog.setLabelText(f"正在停止: {stdout_data.strip()[:50]}")
                            has_data = True
                    except (IOError, BrokenPipeError) as e:
                        print(f"读取stdout时出错: {str(e)}")
                
                # 读取stderr
                if stderr_fd in readable:
                    try:
                        # 读一行或者最多4096字节
                        stderr_data = process.stderr.readline()
                        if stderr_data:
                            all_errors.append(stderr_data)
                            has_data = True
                    except (IOError, BrokenPipeError) as e:
                        print(f"读取stderr时出错: {str(e)}")
                
                return has_data
            
            # 添加超时机制
            start_time = time.time()
            timeout = 15  # 设置最大超时时间为15秒
            
            # 非阻塞方式读取输出，带超时检测
            while process.poll() is None:
                # 检查是否超时
                if time.time() - start_time > timeout:
                    print("停止进程执行超时，强制终止...")
                    process.terminate()  # 先尝试温和终止
                    time.sleep(0.5)
                    if process.poll() is None:  # 如果还没结束
                        process.kill()  # 强制终止
                        print("已强制终止停止脚本进程")
                    break
                
                # 读取输出
                if not read_output():
                    time.sleep(0.1)
                QApplication.processEvents()
            
            # 尝试读取剩余输出，但也加入超时保护
            read_timeout = time.time() + 2  # 最多再读2秒
            while time.time() < read_timeout and read_output():
                QApplication.processEvents()
                pass
            
            # 处理完毕，准备显示结果
            stdout = "".join(all_output)
            stderr = "".join(all_errors)
            
            # 更新进度到100%
            progress_dialog.setValue(100)
            progress_dialog.setLabelText("停止完成")
            QApplication.processEvents()
            time.sleep(0.5)  # 短暂延迟以显示完成状态
            progress_dialog.close()
            
            # 简化UI流程，使用简单的消息框而不是复杂的对话框
            # 在控制台记录所有输出，但不在UI中显示详细信息
            print("停止脚本执行完成，返回码:", process.returncode)
            if stdout:
                print("脚本输出:", stdout.strip())
            if stderr:
                print("错误信息:", stderr.strip())
            
            # 只显示简单的成功消息，减少UI阻塞
            if process.returncode == 0:
                message = "✅ 无人机系统进程已停止"
                QMessageBox.information(self, "停止完成", message)
            else:
                message = "⚠️ 部分进程可能未正常停止，请查看控制台日志"
                QMessageBox.warning(self, "停止警告", message)
            
            # 无论成功与否，都重置UI状态，释放所有资源
            # 更新UI状态，显示系统已停止
            if hasattr(self, 'position_label'):
                self.position_label.setText("Position: (系统已停止)")
            
            # 确保话题订阅器被完全关闭
            if hasattr(self, 'topic_subscriber') and self.topic_subscriber:
                try:
                    self.topic_subscriber.shutdown()
                    self.topic_subscriber = None
                except:
                    pass
            
            # 重置所有UI标签到初始状态
            if hasattr(self, 'altitude_label'):
                self.altitude_label.setText("-- m")
            
            if hasattr(self, 'ground_speed_label'):
                self.ground_speed_label.setText("-- m/s")
            
            if hasattr(self, 'mode_label'):
                self.mode_label.setText("未连接")
            
            if hasattr(self, 'connection_label'):
                self.connection_label.setText("未连接")
                # 不设置字体大小，保持卡片的原始字体设置
                self.connection_label.setStyleSheet("""
                    QLabel {
                        color: #E74C3C;
                        font-weight: bold;
                        background: transparent;
                        border: none;
                        padding: 0px;
                        margin: 0px;
                    }
                """)
            
            if hasattr(self, 'battery_status_label'):
                self.battery_status_label.setText("--%")
            
            if hasattr(self, 'voltage_label'):
                self.voltage_label.setText("-- V")
            
            if hasattr(self, 'pitch_label'):
                self.pitch_label.setText("0.00°")
            
            if hasattr(self, 'roll_label'):
                self.roll_label.setText("0.00°")
            
            if hasattr(self, 'yaw_label'):
                self.yaw_label.setText("0.00°")
            
            # 重置电池图标
            if hasattr(self, 'battery_icon_label'):
                self.battery_icon_label.setPixmap(QPixmap(":/images/icons/battery_100.svg").scaled(24, 24, Qt.KeepAspectRatio, Qt.SmoothTransformation))
            
            # 重置图像显示区域状态
            if hasattr(self, 'image_label'):
                self.image_label.setText("<div style='font-size: 16pt; color: #3498DB; text-align: center; margin-top: 200px;'>系统已停止，请点击\"一键启动\"启动后台程序</div>")
            
            if hasattr(self, 'bird_view_label'):
                self.bird_view_label.setText("<div style='font-size: 14pt; color: #3498DB; text-align: center; margin-top: 100px;'>系统已停止</div>")
            
            # 清除图像数据
            self.camera_image = None
            self.depth_image = None
            self.bird_view_image = None
            
            # 重置话题数据状态标志
            self.topics_with_data = {
                "battery": False,
                "status": False,
                "odometry": False,
                "velocity": False,
                "camera": False,
                "depth": False,
                "bird_view": False,
                "marker": False
            }           
        except Exception as e:
            if 'progress_dialog' in locals() and progress_dialog is not None:
                try:
                    progress_dialog.close()
                except:
                    pass
            error_msg = f"停止无人机系统时出错: {str(e)}"
            QMessageBox.critical(self, "停止失败", error_msg)

    def publishNavigationGoal(self):
        """发布导航目标点到/move_base_simple/goal话题"""
        try:
            # 弹出确认对话框
            reply = QMessageBox.question(self, "确认探索", 
                                     "确定要开始探索模式吗？", 
                                     QMessageBox.Yes | QMessageBox.No,
                                     QMessageBox.No)  # 默认选择"否"
            
            if reply != QMessageBox.Yes:
                # 用户未确认，不执行操作
                return
            
            # 创建目标点消息
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = "world"  # 使用地图坐标系
            goal_msg.header.stamp = rospy.Time.now()
            
            # 设置目标点位置
            goal_msg.pose.position.x = 1.0
            goal_msg.pose.position.y = 1.0
            goal_msg.pose.position.z = 0.7
            
            # 设置目标点朝向（使用默认朝向）
            goal_msg.pose.orientation.x = 0.0
            goal_msg.pose.orientation.y = 0.0
            goal_msg.pose.orientation.z = 0.0
            goal_msg.pose.orientation.w = 1.0
            
            # 创建发布者
            goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
            
            # 稍微延迟，确保发布者连接到订阅者
            rospy.sleep(0.5)
            
            # 发布两次目标点
            goal_pub.publish(goal_msg)
            rospy.loginfo("已发布第一次导航目标点: (1.0, 1.0, 0.7)")
            
            # 稍微延迟发布第二次
            rospy.sleep(1.0)
            
            # 更新时间戳并再次发布
            goal_msg.header.stamp = rospy.Time.now()
            goal_pub.publish(goal_msg)
            rospy.loginfo("已发布第二次导航目标点: (1.0, 1.0, 0.7)")
            
        except Exception as e:
            QMessageBox.critical(self, "发布错误", f"发布导航目标点时出错: {str(e)}")



    def setupRVizOverlay(self):
        """创建悬浮在RViz上方的信息面板 - 独立窗口，但跟随RViz框架移动"""
        # 创建悬浮面板容器 - 独立窗口
        self.rviz_overlay = QWidget()
        self.rviz_overlay.setObjectName("rvizOverlay")
        
        # 设置窗口标志，使其作为工具窗口、无边框并置顶
        self.rviz_overlay.setWindowFlags(Qt.Tool | Qt.FramelessWindowHint)
        
        # 设置窗口背景透明
        self.rviz_overlay.setAttribute(Qt.WA_TranslucentBackground)
        
        # 使用浅黑色背景，30%不透明度，添加较大圆角
        self.rviz_overlay.setStyleSheet("""
            QWidget#rvizOverlay {
                background-color: rgba(26, 32, 44, 0.3);  /* 30%不透明度浅黑色 */
                border-radius: 15px;  /* 较大的圆角 */
            }
            QLabel {
                color: white;
                font-size: 12pt;
                background-color: transparent;
                padding: 2px;
            }
            QLabel.value {
                color: #3498DB;  /* 蓝色值 */
                font-weight: bold;
            }
        """)
        
        # 创建水平布局
        overlay_layout = QHBoxLayout(self.rviz_overlay)
        overlay_layout.setContentsMargins(12, 6, 12, 6)
        overlay_layout.setSpacing(3)  # 进一步减小间距
        
        # 定义要显示的信息项和对应图标
        info_items = [
            {"icon": ":/images/icons/flitghtmode.svg", "value_id": "mode_value"},
            {"icon": ":/images/icons/remotecontrol.svg", "value_id": "rc_value"}, 
            {"icon": ":/images/icons/h.svg", "value_id": "altitude_value", "unit": "m"},
            {"icon": ":/images/icons/d.svg", "value_id": "speed_value", "unit": "m/s"},
            {"icon": ":/images/icons/voltage.svg", "value_id": "voltage_value", "unit": "V"}, 
            {"icon": ":/images/icons/battery_100.svg", "value_id": "battery_value", "unit": "%"}
        ]
        
        # 创建图标和标签
        for i, item in enumerate(info_items):
            # 如果不是第一项，添加更小的间距，但不添加分隔线（保持透明效果）
            if i > 0:
                spacer = QSpacerItem(2, 10, QSizePolicy.Fixed, QSizePolicy.Minimum)
                overlay_layout.addItem(spacer)
            
            # 创建一个水平布局的容器来放置图标和值
            item_container = QWidget()
            item_layout = QHBoxLayout(item_container)
            item_layout.setContentsMargins(0, 0, 0, 0)
            item_layout.setSpacing(2)  # 减小图标和值之间的间距
            
            # 图标
            icon_label = QLabel()
            icon_pixmap = QPixmap(item["icon"]).scaled(20, 20, Qt.KeepAspectRatio, Qt.SmoothTransformation)  # 略微减小图标
            icon_label.setPixmap(icon_pixmap)
            icon_label.setAlignment(Qt.AlignCenter)
            icon_label.setContentsMargins(0, 0, 0, 0)  # 移除内边距
            
            # 为特定图标设置对象名称，方便后续查找
            if "battery" in item["icon"]:
                icon_label.setObjectName("batteryIcon")
                icon_label.setProperty("icon_type", "battery")
            elif "voltage" in item["icon"]:
                icon_label.setObjectName("voltageIcon")
            
            item_layout.addWidget(icon_label)
            
            # 合并值和单位到一个标签
            value_label = QLabel("--" + (" " + item["unit"] if "unit" in item else ""))
            value_label.setProperty("class", "value")
            value_label.setProperty("unit", item.get("unit", ""))  # 保存单位信息
            value_label.setContentsMargins(0, 0, 0, 0)  # 移除内边距
            setattr(self, item["value_id"], value_label)
            item_layout.addWidget(value_label)
            
            overlay_layout.addWidget(item_container)
        
        # 设置固定高度和宽度
        self.rviz_overlay.setFixedHeight(40)
        self.rviz_overlay.setFixedWidth(750)  # 固定宽度为750像素
        
        # 更新位置的函数
        def updateOverlayPosition():
            try:
                if hasattr(self, 'frame') and self.frame and hasattr(self, 'rviz_overlay') and self.rviz_overlay:
                    # 确保RViz框架已经完成布局更新
                    self.frame.update()
                    QApplication.processEvents()

                    # 直接使用RViz框架的几何信息，因为它已经考虑了分割器的布局
                    frame_rect = self.frame.geometry()
                    frame_pos = self.frame.mapToGlobal(QPoint(0, 0))

                    # 检查几何信息是否有效
                    if frame_rect.width() > 0 and frame_rect.height() > 0:
                        # 居中显示在RViz框架上方
                        x_pos = frame_pos.x() + (frame_rect.width() - self.rviz_overlay.width()) // 2
                        y_pos = frame_pos.y() + 20

                        # 移动窗口
                        self.rviz_overlay.move(x_pos, y_pos)

                        # 确保窗口可见
                        if not self.rviz_overlay.isVisible():
                            self.rviz_overlay.show()
            except Exception as e:
                print(f"更新信息条位置时出错: {e}")
        
        # 将函数保存为类实例方法，以便后续修改
        self.updateOverlayPosition = updateOverlayPosition

        # 立即显示悬浮面板
        self.rviz_overlay.show()

        # 注意：悬浮组件的位置更新已合并到主更新循环中，减少定时器数量
        # 初始位置更新
        QTimer.singleShot(100, updateOverlayPosition)

    def updateOverlayData(self):
        """更新信息条数据"""
        if not all(hasattr(self, attr) for attr in ['mode_value', 'altitude_value', 'speed_value', 'battery_value', 'voltage_value', 'rc_value']):
            return  # 确保UI已初始化
        
        try:
            # 更新模式
            if hasattr(self, 'mode_label'):
                mode_text = self.mode_label.text().split(" ")[0] if " " in self.mode_label.text() else self.mode_label.text()
                self.mode_value.setText(mode_text)
            
            # 更新高度
            if hasattr(self, 'altitude_label'):
                # 从格式为"0.0000 m"的文本中提取数值部分
                height_text = self.altitude_label.text()
                if ' ' in height_text:
                    height_value = height_text.split(' ')[0]
                    # 格式化为最多2位小数
                    try:
                        height_value = f"{float(height_value):.2f}"
                    except:
                        pass
                    # 获取单位并添加到文本中
                    unit = self.altitude_value.property("unit") or ""
                    self.altitude_value.setText(f"{height_value} {unit}".strip())
                else:
                    # 获取单位并添加到文本中
                    unit = self.altitude_value.property("unit") or ""
                    self.altitude_value.setText(f"{height_text} {unit}".strip())
            
            # 更新速度
            if hasattr(self, 'ground_speed_label'):
                # 从格式为"0.0000 m/s"的文本中提取数值部分
                speed_text = self.ground_speed_label.text()
                if ' ' in speed_text:
                    speed_value = speed_text.split(' ')[0]
                    # 格式化为最多2位小数
                    try:
                        speed_value = f"{float(speed_value):.2f}"
                    except:
                        pass
                    # 获取单位并添加到文本中
                    unit = self.speed_value.property("unit") or ""
                    self.speed_value.setText(f"{speed_value} {unit}".strip())
                else:
                    # 获取单位并添加到文本中
                    unit = self.speed_value.property("unit") or ""
                    self.speed_value.setText(f"{speed_text} {unit}".strip())
            
            # 更新电池电量
            if hasattr(self, 'battery_percentage'):
                battery = f"{self.battery_percentage:.1f}" if isinstance(self.battery_percentage, (int, float)) else "--"
                # 获取单位并添加到文本中
                unit = self.battery_value.property("unit") or ""
                self.battery_value.setText(f"{battery} {unit}".strip())
                
                # 根据电量更新电池图标
                battery_value = float(battery) if battery != "--" else 100
                if battery_value <= 15:
                    icon_path = ":/images/icons/battery_0.svg"
                elif battery_value <= 50:
                    icon_path = ":/images/icons/battery_50.svg"
                elif battery_value <= 75:
                    icon_path = ":/images/icons/battery_75.svg"
                else:
                    icon_path = ":/images/icons/battery_100.svg"
                    
                # 直接通过对象名称查找电池图标并更新
                battery_icon = self.rviz_overlay.findChild(QLabel, "batteryIcon")
                if battery_icon:
                    battery_icon.setPixmap(QPixmap(icon_path).scaled(22, 22, Qt.KeepAspectRatio, Qt.SmoothTransformation))
                else:
                    # 备用方法：搜索所有标签
                    for widget in self.rviz_overlay.findChildren(QLabel):
                        if widget.property("icon_type") == "battery" or "battery" in str(widget.objectName()):
                            widget.setPixmap(QPixmap(icon_path).scaled(20, 20, Qt.KeepAspectRatio, Qt.SmoothTransformation))
                            widget.setObjectName("batteryIcon")  # 确保设置了名称
                            break
            
            # 更新电压
            if hasattr(self, 'battery_voltage'):
                voltage = f"{self.battery_voltage:.2f}" if isinstance(self.battery_voltage, (int, float)) else "--"
                # 获取单位并添加到文本中
                unit = self.voltage_value.property("unit") or ""
                self.voltage_value.setText(f"{voltage} {unit}".strip())
            
            # 更新遥控器连接状态
            if hasattr(self, 'topic_subscriber') and self.topic_subscriber:
                # 检查遥控器话题是否活跃
                rc_active = self.topic_subscriber.is_topic_active('rc_input')
                # 检查最新数据
                rc_data = self.topic_subscriber.get_latest_data('rc_input')
                
                # 确定连接状态
                if rc_active and rc_data and 'channels' in rc_data and len(rc_data['channels']) > 0:
                    self.rc_value.setText("已连接")
                    self.rc_value.setStyleSheet("color: #2ECC71; font-weight: bold;")  # 绿色表示已连接
                else:
                    self.rc_value.setText("未连接")
                    self.rc_value.setStyleSheet("color: #E74C3C; font-weight: bold;")  # 红色表示未连接
                
        except Exception as e:
            print(f"更新信息面板数据时出错: {str(e)}")

    def silentStopDroneSystem(self):
        """静默关闭无人机系统，不显示任何对话框"""
        try:
            # 立即停止所有进程监控定时器，避免重复弹出错误消息
            for timer_attr in ['check_process_timer', 'check_process2_timer', 'check_process3_timer', 'process_monitor_timer']:
                if hasattr(self, timer_attr):
                    timer = getattr(self, timer_attr)
                    if timer and timer.isActive():
                        timer.stop()
                        print(f"已停止{timer_attr}")
            
            # 如果有任何话题订阅器在运行，先关闭它
            if self.topic_subscriber:
                try:
                    self.topic_subscriber.shutdown()
                    self.topic_subscriber = None
                    # 重置话题数据状态，确保下次启动时正确初始化
                    self.topics_with_data = {
                        "battery": False,
                        "status": False,
                        "odometry": False,
                        "velocity": False,
                        "camera": False,
                        "depth": False,
                        "bird_view": False,
                        "marker": False
                    }
                    print("已关闭话题订阅器并重置话题数据状态")
                except Exception as e:
                    print(f"关闭话题订阅器时出错: {str(e)}")
            
            # 检查是否有单独启动的进程需要关闭
            if hasattr(self, 'processes') and self.processes:
                # 按照启动的相反顺序终止进程
                process_order = ["ball_tracker", "planner", "px4ctrl", "detector", "vins", "mavros", "camera"]
                
                for process_name in process_order:
                    if process_name in self.processes and self.processes[process_name]:
                        try:
                            # 终止进程
                            self.processes[process_name].terminate()
                            # 给进程一点时间自行退出
                            start_time = time.time()
                            while time.time() - start_time < 1:  # 最多等待1秒
                                if self.processes[process_name].poll() is not None:
                                    # 进程已结束
                                    break
                                time.sleep(0.1)
                            
                            # 如果进程仍未退出，强制杀死
                            if self.processes[process_name].poll() is None:
                                self.processes[process_name].kill()
                                
                            print(f"已停止{process_name}进程")
                        except Exception as e:
                            print(f"停止{process_name}进程时出错: {str(e)}")
                
                # 清空进程列表
                self.processes = {}
            
            # 使用强大的终止机制，确保所有相关进程都被终止
            # 使用全局常量避免重复定义
            process_patterns = PROCESS_PATTERNS
            
            # 使用pkill强制终止每个模式的进程
            for pattern in process_patterns:
                try:
                    # 使用pgrep检查进程是否存在
                    check_process = subprocess.run(
                        f"pgrep -f \"{pattern}\"", 
                        shell=True, 
                        stdout=subprocess.PIPE, 
                        stderr=subprocess.PIPE
                    )
                    
                    if check_process.returncode == 0:  # 进程存在
                        # 使用pkill -9 强制终止
                        kill_cmd = f"pkill -9 -f \"{pattern}\""
                        subprocess.run(kill_cmd, shell=True)
                        print(f"已终止进程：{pattern}")
                    else:
                        print(f"未找到进程：{pattern}")
                except Exception as e:
                    print(f"终止进程 {pattern} 时出错: {str(e)}")
            
            # 保留roscore，但清理其他所有ROS节点
            try:
                # 先检查ROS环境是否正常
                rosnode_check = subprocess.run(
                    "rosnode list", 
                    shell=True, 
                    stdout=subprocess.PIPE, 
                    stderr=subprocess.PIPE, 
                    text=True, 
                    timeout=5
                )
                
                if rosnode_check.returncode == 0:
                    # 获取节点列表
                    nodes = rosnode_check.stdout.strip().split('\n')
                    # 过滤掉rosout和master相关节点
                    nodes_to_kill = [node for node in nodes if not any(x in node for x in ['/rosout', '/master'])]
                    
                    if nodes_to_kill:
                        # 每个节点单独终止
                        for node in nodes_to_kill:
                            kill_cmd = f"rosnode kill {node}"
                            subprocess.run(kill_cmd, shell=True, timeout=2)
                            print(f"已清理节点: {node}")
                    
                    print("已清理所有非核心ROS节点")
                else:
                    print("ROS环境可能未启动或异常")
            except Exception as e:
                print(f"清理ROS节点时出错: {str(e)}")
                
        except Exception as e:
            print(f"静默停止无人机系统时出错: {str(e)}")
            
    def setupCompass(self):
        """创建指南针组件并添加到RViz右下角，独立窗口但跟随RViz框架移动"""
        # 导入指南针组件
        from dashboard import CompassWidget
        
        # 创建指南针组件
        self.compass = CompassWidget(parent=self)  # 传入self作为参考，但不作为父组件
        
        # 定义位置更新函数
        def updateCompassPosition():
            try:
                if hasattr(self, 'frame') and self.frame and hasattr(self, 'compass') and self.compass:
                    # 确保RViz框架已经完成布局更新
                    self.frame.update()
                    QApplication.processEvents()

                    # 直接使用RViz框架的几何信息
                    frame_rect = self.frame.geometry()
                    frame_pos = self.frame.mapToGlobal(QPoint(0, 0))

                    # 检查几何信息是否有效
                    if frame_rect.width() > 0 and frame_rect.height() > 0:
                        # 放在RViz框架的右下角，增加边距让它更靠左上方一些
                        margin_x = 60  # 增加水平边距
                        margin_y = 50  # 增加垂直边距
                        x_pos = frame_pos.x() + frame_rect.width() - self.compass.width() - margin_x
                        y_pos = frame_pos.y() + frame_rect.height() - self.compass.height() - margin_y

                        # 移动窗口
                        self.compass.move(x_pos, y_pos)

                        # 确保窗口可见
                        if not self.compass.isVisible():
                            self.compass.show()
            except Exception as e:
                print(f"更新指南针位置时出错: {e}")
        
        # 将函数保存为类实例方法，以便后续修改
        self.updateCompassPosition = updateCompassPosition
        
        # 注意：指南针的位置和数据更新已合并到主更新循环中
        # 初始位置更新
        QTimer.singleShot(100, updateCompassPosition)

    def setupAttitudeWidget(self):
        """创建姿态指示器组件并添加到RViz右下角，独立窗口但跟随RViz框架移动"""
        # 导入姿态指示器组件
        from dashboard import AttitudeIndicatorWidget
        
        # 创建姿态指示器组件
        self.attitude_widget = AttitudeIndicatorWidget(parent=self)  # 传入self作为参考，但不作为父组件
        
        # 定义位置更新函数 - 直接基于RViz框架位置计算
        def updateAttitudeWidgetPosition():
            try:
                if hasattr(self, 'frame') and self.frame and hasattr(self, 'attitude_widget') and self.attitude_widget:
                    # 确保RViz框架已经完成布局更新
                    self.frame.update()
                    QApplication.processEvents()

                    # 直接使用RViz框架的几何信息
                    frame_rect = self.frame.geometry()
                    frame_pos = self.frame.mapToGlobal(QPoint(0, 0))

                    # 检查几何信息是否有效
                    if frame_rect.width() > 0 and frame_rect.height() > 0:
                        # 放在RViz框架的右下角位置，与指南针并排
                        margin_x = 260  # 增加水平边距，放在指南针左侧
                        margin_y = 50   # 垂直边距与指南针相同
                        x_pos = frame_pos.x() + frame_rect.width() - self.attitude_widget.width() - margin_x
                        y_pos = frame_pos.y() + frame_rect.height() - self.attitude_widget.height() - margin_y

                        # 确保不会移出RViz框架左侧
                        min_x = frame_pos.x() + 20
                        if x_pos < min_x:
                            # 如果左侧空间不足，放在上方
                            x_pos = frame_pos.x() + frame_rect.width() - self.attitude_widget.width() - 60
                            y_pos = frame_pos.y() + frame_rect.height() - self.attitude_widget.height() - 240

                        # 移动窗口
                        self.attitude_widget.move(x_pos, y_pos)

                        # 确保窗口可见
                        if not self.attitude_widget.isVisible():
                            self.attitude_widget.show()
            except Exception as e:
                print(f"更新姿态指示器位置时出错: {e}")
        
        # 将函数保存为类实例方法，以便后续修改
        self.updateAttitudeWidgetPosition = updateAttitudeWidgetPosition

        # 注意：姿态指示器的位置更新已合并到主更新循环中，减少定时器数量
        # 初始位置更新
        QTimer.singleShot(100, updateAttitudeWidgetPosition)

    def setupAllOverlays(self):
        """同时创建所有悬浮窗口组件，确保它们同时显示"""
        # 暂时禁用鼠标跟踪，避免在创建悬浮窗口时触发左侧栏显示
        old_enable_state = getattr(self, 'enable_sidebar_hover', False)
        self.enable_sidebar_hover = False
        
        # 1. 创建RViz悬浮信息面板
        self.setupRVizOverlay()
        
        # 2. 创建指南针组件
        self.setupCompass()
        
        # 3. 创建姿态指示器组件
        self.setupAttitudeWidget()
        
        # 恢复原来的鼠标跟踪状态
        self.enable_sidebar_hover = old_enable_state
        
    # handleCloseEvent方法已移至closeEvent方法下实现

    def autoHideSidebar(self):
        """启动后自动隐藏左侧栏和右侧栏"""
        self.toggleSidebar(hide=True, animate=True)
        self.toggleRightSidebar(hide=True, animate=True)
        
    def checkMousePosition(self):
        """检查鼠标位置，根据位置自动显示或隐藏左右侧栏"""
        # 如果鼠标跟踪未启用，直接返回
        if not hasattr(self, 'enable_sidebar_hover') or not self.enable_sidebar_hover:
            return
            
        try:
            # 获取鼠标当前位置
            cursor_pos = QCursor.pos()
            # 将全局坐标转换为窗口坐标
            local_pos = self.mapFromGlobal(cursor_pos)
            
            # 定义左右侧敏感区域宽度
            sensitivity_width = 20
            
            # 获取窗口宽度
            window_width = self.width()
            
            # 检查是否在左侧敏感区域内
            in_left_sensitive_area = 0 <= local_pos.x() <= sensitivity_width
            
            # 检查是否在右侧敏感区域内
            in_right_sensitive_area = window_width - sensitivity_width <= local_pos.x() <= window_width
            
            # 检查是否在窗口内
            in_window = self.rect().contains(local_pos)
            
            # 计算左侧栏区域
            sidebar_width = self.left_sidebar.width() if self.left_sidebar.isVisible() else 0
            in_sidebar_area = 0 <= local_pos.x() <= sidebar_width and in_window
            
            # 计算右侧栏区域
            right_sidebar_width = self.right_sidebar.width() if self.right_sidebar.isVisible() else 0
            in_right_sidebar_area = window_width - right_sidebar_width <= local_pos.x() <= window_width and in_window
            
            # 处理左侧栏 - 仅在未固定时进行自动显示/隐藏
            if not self.left_sidebar_pinned:
                # 如果鼠标在窗口内的左侧敏感区域，显示左侧栏
                if in_window and in_left_sensitive_area and not self.sidebar_expanded:
                    self.toggleSidebar(hide=False, animate=True)
                
                # 如果鼠标不在左侧敏感区域且不在左侧栏内，隐藏左侧栏
                elif self.sidebar_expanded and not in_sidebar_area and not in_left_sensitive_area:
                    self.toggleSidebar(hide=True, animate=True)
            
            # 处理右侧栏 - 仅在未固定时进行自动显示/隐藏
            if not self.right_sidebar_pinned:
                # 如果鼠标在窗口内的右侧敏感区域，显示右侧栏
                if in_window and in_right_sensitive_area and not self.right_sidebar_expanded:
                    self.toggleRightSidebar(hide=False, animate=True)
                
                # 如果鼠标不在右侧敏感区域且不在右侧栏内，隐藏右侧栏
                elif self.right_sidebar_expanded and not in_right_sidebar_area and not in_right_sensitive_area:
                    self.toggleRightSidebar(hide=True, animate=True)
                
        except Exception as e:
            print(f"检查鼠标位置时出错: {str(e)}")
            # 错误时停止定时器以防止继续出错
            self.sidebar_hover_timer.stop()

    def setupAllOverlaysAndHideSidebar(self):
        """设置所有悬浮窗口并在完成后隐藏左右侧栏"""
        # 先设置所有悬浮窗口
        self.setupAllOverlays()
        
        # 延迟500ms后隐藏左右侧栏，确保悬浮窗口已完全显示
        QTimer.singleShot(500, self.finalizeStartup)
    
    def finalizeStartup(self):
        """完成启动过程，隐藏左右侧栏并启用鼠标跟踪"""
        # 隐藏左侧栏和右侧栏
        self.toggleSidebar(hide=True, animate=True)
        self.toggleRightSidebar(hide=True, animate=True)
        
        # 确保两侧的固定状态为未固定
        self.left_sidebar_pinned = False
        self.right_sidebar_pinned = False
        
        # 恢复按钮样式为默认
        self.toggle_sidebar_btn.setStyleSheet("""
            QPushButton {
                background-color: #1A202C;
                border: none;
                border-radius: 0;
                padding: 2px;
            }
            QPushButton:hover {
                background-color: #3498DB;
            }
            QPushButton:pressed {
                background-color: #2980B9;
            }
        """)
        self.toggle_right_sidebar_btn.setStyleSheet("""
            QPushButton {
                background-color: #1A202C;
                border: none;
                border-radius: 0;
                padding: 2px;
            }
            QPushButton:hover {
                background-color: #3498DB;
            }
            QPushButton:pressed {
                background-color: #2980B9;
            }
        """)
        
        # 延迟300ms后启用鼠标跟踪，避免动画过程中触发鼠标跟踪
        QTimer.singleShot(300, self.enableMouseTracking)
    
    def enableMouseTracking(self):
        """启用鼠标跟踪"""
        self.enable_sidebar_hover = True
        print("鼠标跟踪已启用")

    def on_position_table_cell_clicked(self, row, column):
        """处理位置表格单元格点击事件"""
        try:
            # 只处理截图列的点击
            if column == 4:  # 截图列
                # 安全地获取ball_id
                item = self.position_table.item(row, 0)
                if item is None:
                    print(f"表格行 {row} 的ID列为空")
                    return

                try:
                    ball_id = int(item.text())
                except ValueError:
                    print(f"无法解析ball_id: {item.text()}")
                    return

                # 检查截图列的内容
                screenshot_item = self.position_table.item(row, 4)
                if screenshot_item is None or screenshot_item.text() != "查看截图":
                    print(f"小球 {ball_id} 没有可用的截图")
                    return

                if ball_id in self.ball_screenshots:
                    # 显示截图对话框
                    self.show_screenshot_dialog(ball_id)
                else:
                    print(f"未找到小球 {ball_id} 的截图数据")

        except Exception as e:
            print(f"处理表格点击事件时出错: {str(e)}")
            import traceback
            traceback.print_exc()
    
    def show_screenshot_dialog(self, ball_id):
        """显示小球截图对话框"""
        try:
            if ball_id not in self.ball_screenshots:
                print(f"未找到小球 {ball_id} 的截图数据")
                return

            # 获取截图数据
            screenshot_data = self.ball_screenshots[ball_id]

            # 创建对话框
            dialog = QDialog(self)

            # 保存对话框引用，防止被垃圾回收
            if not hasattr(self, 'screenshot_dialogs'):
                self.screenshot_dialogs = []
            self.screenshot_dialogs.append(dialog)
            dialog.setWindowTitle(f"小球ID {ball_id} 截图")
            dialog.setMinimumSize(640, 480)
            dialog.setModal(False)  # 设置为非模态对话框，允许自由拖动

            # 设置窗口标志，确保可以拖动和调整大小
            dialog.setWindowFlags(Qt.Window | Qt.WindowTitleHint | Qt.WindowCloseButtonHint |
                                Qt.WindowMinMaxButtonsHint | Qt.WindowSystemMenuHint)

            # 设置窗口可调整大小
            dialog.setSizeGripEnabled(True)

            # 创建布局
            layout = QVBoxLayout(dialog)

            # 创建图像标签
            image_label = QLabel()
            image_label.setAlignment(Qt.AlignCenter)
            image_label.setStyleSheet("border: 1px solid #3498DB; background-color: #1E2330;")

            # 安全地加载和显示图像
            success = False
            if "path" in screenshot_data and os.path.exists(screenshot_data["path"]):
                try:
                    # 使用OpenCV安全地加载图像
                    cv_image = cv2.imread(screenshot_data["path"])
                    if cv_image is not None:
                        # 转换颜色格式从BGR到RGB
                        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                        height, width, channel = rgb_image.shape

                        # 创建QImage时确保数据连续性
                        bytes_per_line = 3 * width
                        q_image = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)

                        # 创建QPixmap的副本以确保数据安全
                        pixmap = QPixmap.fromImage(q_image.copy())

                        if not pixmap.isNull():
                            # 缩放图像以适应对话框
                            scaled_pixmap = pixmap.scaled(600, 400, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                            image_label.setPixmap(scaled_pixmap)
                            success = True
                            print(f"成功加载小球 {ball_id} 的截图")
                        else:
                            print(f"创建QPixmap失败: {screenshot_data['path']}")
                    else:
                        print(f"OpenCV无法读取图像文件: {screenshot_data['path']}")
                except Exception as e:
                    print(f"加载截图时出错: {str(e)}")

            if not success:
                # 如果加载失败，显示错误信息
                image_label.setText(f"""
                    <div style='
                        color: #E74C3C;
                        font-size: 16pt;
                        text-align: center;
                        padding: 50px;
                    '>
                        截图加载失败<br>
                        小球ID: {ball_id}
                    </div>
                """)

            # 添加到布局
            layout.addWidget(image_label)

            # 添加关闭按钮
            close_button = QPushButton("关闭")
            close_button.setStyleSheet("""
                QPushButton {
                    background-color: #3498DB;
                    color: white;
                    border: none;
                    border-radius: 4px;
                    padding: 8px 16px;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background-color: #2980B9;
                }
            """)
            # 连接关闭按钮，确保正确清理对话框
            def close_dialog():
                if hasattr(self, 'screenshot_dialogs') and dialog in self.screenshot_dialogs:
                    self.screenshot_dialogs.remove(dialog)
                dialog.close()

            close_button.clicked.connect(close_dialog)
            layout.addWidget(close_button)

            # 设置对话框关闭事件处理
            def on_dialog_close(event):
                if hasattr(self, 'screenshot_dialogs') and dialog in self.screenshot_dialogs:
                    self.screenshot_dialogs.remove(dialog)
                event.accept()

            dialog.closeEvent = on_dialog_close

            # 显示对话框（非阻塞方式）
            dialog.show()
            dialog.raise_()  # 将窗口提到前台
            dialog.activateWindow()  # 激活窗口

        except Exception as e:
            print(f"显示截图对话框时出错: {str(e)}")
            # 显示错误对话框
            error_dialog = QDialog(self)
            error_dialog.setWindowTitle("错误")
            error_dialog.setMinimumSize(300, 150)
            error_layout = QVBoxLayout(error_dialog)
            error_label = QLabel(f"显示截图时出错:\n{str(e)}")
            error_layout.addWidget(error_label)
            error_button = QPushButton("确定")
            error_button.clicked.connect(error_dialog.accept)
            error_layout.addWidget(error_button)
            error_dialog.exec_()

    # 注意：closeEvent方法已在上面优化实现，删除重复代码

## Start the Application
## ^^^^^^^^^^^^^^^^^^^^^
##
## That is just about it.  All that's left is the standard Qt
## top-level application code: create a QApplication, instantiate our
## class, and start Qt's main event loop (app.exec_()).
# 已移除check_rosdep_status函数，因为不需要检测rosdep

def check_and_start_roscore():
    """检查roscore是否运行，如果没有则启动它"""
    import subprocess
    import time
    import os
    
    # 检查roscore是否已在运行
    try:
        # 尝试使用rostopic list检查ROS master是否运行
        check_process = subprocess.Popen(['rostopic', 'list'], 
                                        stdout=subprocess.PIPE, 
                                        stderr=subprocess.PIPE)
        _, stderr = check_process.communicate(timeout=2)
        
        if check_process.returncode != 0:
            print("未检测到roscore运行，正在自动启动roscore...")
            
            # 启动roscore并在后台运行
            roscore_process = subprocess.Popen(['roscore'], 
                                            stdout=subprocess.PIPE, 
                                            stderr=subprocess.PIPE)
            
            # 等待roscore启动
            print("等待roscore启动...")
            time.sleep(3)  # 给roscore一些启动时间
            
            # 存储roscore进程ID以便在应用程序退出时关闭
            os.environ['ROSCORE_PID'] = str(roscore_process.pid)
            print(f"roscore已启动，PID: {roscore_process.pid}")
            return True
        else:
            print("已检测到roscore正在运行")
            return False
    except Exception as e:
        print(f"检查或启动roscore时出错: {str(e)}")
        return False

def set_serial_permissions():
    """设置串口设备权限，使用sudo chmod 777 /dev/ttyACM0，密码为1"""
    try:
        import subprocess
        
        print("正在设置串口设备权限...")
        # 执行sudo命令修改/dev/ttyACM0的权限，通过管道提供密码
        process = subprocess.Popen(
            ['sudo', '-S', 'chmod', '777', '/dev/ttyACM0'], 
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE,
            universal_newlines=True
        )
        
        # 提供管理员密码"1"
        stdout, stderr = process.communicate(input="1\n")
        
        if process.returncode == 0:
            print("串口设备权限设置成功")
            return True
        else:
            print(f"设置串口设备权限失败，错误信息: {stderr}")
            return False
    except Exception as e:
        print(f"设置串口设备权限时出错: {str(e)}")
        return False



if __name__ == '__main__':
    # 设置串口权限
    set_serial_permissions()
    
    app = QApplication(sys.argv)
    
    # 设置应用程序图标
    app_icon = QIcon("logo.png")
    app.setWindowIcon(app_icon)
    
    # 确保应用程序支持中文（如果QTextCodec可用）
    if QTextCodec is not None:
        try:
            QTextCodec.setCodecForLocale(QTextCodec.codecForName("UTF-8"))
        except Exception as e:
            print(f"设置编码时出错: {e}")
    
    # 检查并自动启动roscore
    check_and_start_roscore()
    
    # 初始化ROS节点
    try:
        rospy.init_node('myviz', anonymous=True)
        print("成功初始化ROS节点: myviz")
    except Exception as e:
        print(f"警告: ROS节点初始化失败: {str(e)}")
    
    # 创建主窗口
    try:
        myviz = MyViz()
        print("主窗口创建成功")
    except Exception as e:
        print(f"创建主窗口时出错: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

    # 不要使用自定义布局，避免"already has a layout"错误
    # myviz.setLayout(main_layout) # 删除这一行

    # 获取可用屏幕区域（考虑任务栏/面板）
    desktop = QDesktopWidget()
    available_geometry = desktop.availableGeometry(desktop.primaryScreen())
    width = available_geometry.width()
    height = available_geometry.height()
    print(f"可用屏幕区域: {width}x{height}, 位置: ({available_geometry.x()}, {available_geometry.y()})")

    # 设置窗口为最大化模式启动（保留标题栏）
    myviz.showMaximized()  # 使用最大化模式，保留窗口控制按钮

    # 延迟设置分割器尺寸，确保窗口已完全显示
    QTimer.singleShot(500, myviz.setupAdaptiveSplitterSizes)
    
    # 启动Qt事件循环
    try:
        exit_code = app.exec_()
        
        # 关闭话题订阅器
        if hasattr(myviz, 'topic_subscriber') and myviz.topic_subscriber:
            myviz.topic_subscriber.shutdown()
            print("已关闭话题订阅器")
        
        # 关闭所有话题日志窗口
        try:
            from topic_logger import TopicLoggerDialog
            TopicLoggerDialog.close_all_windows()
            print("已关闭所有话题日志窗口")
        except Exception as e:
            print(f"关闭话题日志窗口时出错: {str(e)}")
        
        # 如果roscore是由本程序启动的，关闭它
        if 'ROSCORE_PID' in os.environ:
            try:
                roscore_pid = int(os.environ['ROSCORE_PID'])
                import signal
                import psutil
                
                print(f"正在关闭自动启动的roscore（PID: {roscore_pid}）...")
                try:
                    # 检查进程是否存在
                    if psutil.pid_exists(roscore_pid):
                        p = psutil.Process(roscore_pid)
                        # 发送SIGINT信号
                        p.send_signal(signal.SIGINT)
                        
                        # 使用psutil等待进程结束，设置较短的超时时间
                        try:
                            p.wait(timeout=3)  # 等待最多3秒
                            print("roscore已正常关闭")
                        except psutil.TimeoutExpired:
                            # 如果超时，强制结束进程
                            print("关闭roscore超时，强制终止...")
                            p.kill()
                            print("roscore已强制关闭")
                    else:
                        print(f"找不到PID为{roscore_pid}的进程，可能已关闭")
                except psutil.NoSuchProcess:
                    print(f"找不到PID为{roscore_pid}的进程，可能已关闭")
            except Exception as e:
                print(f"关闭roscore时出错: {str(e)}，继续执行退出流程")
            
        sys.exit(exit_code)
    except KeyboardInterrupt:
        # 处理Ctrl+C中断
        if hasattr(myviz, 'topic_subscriber') and myviz.topic_subscriber:
            myviz.topic_subscriber.shutdown()
            print("已关闭话题订阅器")
            
        # 关闭所有话题日志窗口
        try:
            from topic_logger import TopicLoggerDialog
            TopicLoggerDialog.close_all_windows()
            print("已关闭所有话题日志窗口")
        except Exception as e:
            print(f"关闭话题日志窗口时出错: {str(e)}")
        
        # 如果roscore是由本程序启动的，关闭它
        if 'ROSCORE_PID' in os.environ:
            try:
                roscore_pid = int(os.environ['ROSCORE_PID'])
                import signal
                import psutil
                
                print(f"正在关闭自动启动的roscore（PID: {roscore_pid}）...")
                try:
                    # 检查进程是否存在
                    if psutil.pid_exists(roscore_pid):
                        p = psutil.Process(roscore_pid)
                        # 发送SIGINT信号
                        p.send_signal(signal.SIGINT)
                        
                        # 使用psutil等待进程结束，设置较短的超时时间
                        try:
                            p.wait(timeout=3)  # 等待最多3秒
                            print("roscore已正常关闭")
                        except psutil.TimeoutExpired:
                            # 如果超时，强制结束进程
                            print("关闭roscore超时，强制终止...")
                            p.kill()
                            print("roscore已强制关闭")
                    else:
                        print(f"找不到PID为{roscore_pid}的进程，可能已关闭")
                except psutil.NoSuchProcess:
                    print(f"找不到PID为{roscore_pid}的进程，可能已关闭")
            except Exception as e:
                print(f"关闭roscore时出错: {str(e)}，继续执行退出流程")
            
        sys.exit(0)