#!/usr/bin/env python
# -*- coding: utf-8 -*-
## First we start with the standard ros Python import line:
import roslib; roslib.load_manifest('rviz_python_tutorial')
import rospy
from geometry_msgs.msg import PoseStamped

## Then load sys to get sys.argv.
import sys
import time
import os
import json
import random
import math
import numpy as np
import cv2
import subprocess  # 添加subprocess模块用于执行命令

# 导入psutil用于进程管理
try:
    import psutil
except ImportError:
    print("警告: 未能导入psutil库，进程管理功能将受限")
    # 尝试自动安装psutil
    try:
        subprocess.call([sys.executable, "-m", "pip", "install", "psutil"])
        import psutil
        print("已成功安装psutil库")
    except:
        print("自动安装psutil失败，请手动安装: pip install psutil")

## 导入我们创建的话题订阅模块、仪表盘组件和话题日志组件
try:
    from topics_subscriber import TopicsSubscriber
except ImportError:
    print("无法导入topics_subscriber模块")
    TopicsSubscriber = None

try:
    from dashboard import DashBoard, UIButton
except ImportError:
    print("无法导入dashboard模块")
    DashBoard = None
    UIButton = None
    
try:
    from topic_logger import TopicLogger
except ImportError:
    print("无法导入topic_logger模块")
    TopicLogger = None

## Next import all the Qt bindings into the current namespace, for
## convenience.  This uses the "python_qt_binding" package which hides
## differences between PyQt and PySide, and works if at least one of
## the two is installed.  The RViz Python bindings use
## python_qt_binding internally, so you should use it here as well.
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

# 确保QTimer和QPropertyAnimation可用
from python_qt_binding.QtCore import QTimer, QPropertyAnimation, QEasingCurve

## 导入生成的资源文件（图标和图片资源）
try:
    import images_rc
except ImportError:
    print("警告: 无法导入images_rc资源文件，请确保已使用pyrcc5编译资源文件")

## Finally import the RViz bindings themselves.
from rviz import bindings as rviz

## The MyViz class is the main container widget.
class MyViz(QMainWindow):  # 使用QMainWindow替代QWidget

    ## MyViz Constructor
    ## ^^^^^^^^^^^^^^^^^
    ##
    ## Its constructor creates and configures all the component widgets:
    ## frame, thickness_slider, top_button, and side_button, and adds them
    ## to layouts.
    def __init__(self):
        QMainWindow.__init__(self)  # 初始化QMainWindow
        
        # 电池状态变量
        self.battery_percentage = 100.0
        self.battery_voltage = 12.0  # 默认电压值
        
        # 设置中文字体支持
        font = QFont("WenQuanYi Micro Hei", 10)
        QApplication.setFont(font)
        
        # 图像显示相关变量
        self.camera_image = None
        self.depth_image = None
        self.bird_view_image = None
        
        # 姿态数据
        self.pitch = 0
        self.roll = 0
        
        # 存储已检测到的标记点ID，避免重复添加
        self.detected_markers = set()
        
        # 话题数据标志，标记话题是否有实际数据
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
        
        # 添加关闭事件处理
        self.closeEvent = self.handleCloseEvent
        
        # 设置窗口标题
        self.setWindowTitle("无人机自主搜索系统")
        
        # 创建中央控件
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        
        # 添加窗口resize事件处理
        self.resizeEvent = self.onResize
        
        # 设置窗口样式，使用黑蓝色调
        self.setStyleSheet("""
            QWidget {
                background-color: #1E2330;
                color: #FFFFFF;
            }
            QPushButton {
                background-color: #2C3E50;
                color: #FFFFFF;
                border: none;
                border-radius: 4px;
                padding: 6px 12px;
                font-weight: bold;
                min-width: 120px;
                min-height: 30px;
            }
            QPushButton:hover {
                background-color: #3498DB;
            }
            QPushButton:pressed {
                background-color: #2980B9;
            }
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
            QLabel {
                font-size: 12pt;
                font-weight: bold;
                padding: 5px;
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
            QToolButton {
                background-color: #2C3E50;
                color: #FFFFFF;
                border: none;
                border-radius: 4px;
                padding: 5px;
                min-width: 30px;
                min-height: 30px;
            }
            QToolButton:hover {
                background-color: #3498DB;
            }
            QToolButton:pressed {
                background-color: #2980B9;
            }
            QStatusBar {
                background-color: #1A202C;
                color: #FFFFFF;
            }
        """)
        
        # 初始化话题订阅器变量（确保此变量先被定义）
        self.topic_subscriber = None
        
        # 设置RViz显示
        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath("")
        self.frame.initialize()
        
        # 读取配置文件
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile(config, "my_config.rviz")
        self.frame.load(config)
        
        # 继续其他初始化...
        
        # 初始化日志窗口
        self.log_window = None
        
        # 添加标志，控制是否启用鼠标跟踪和侧栏是否固定
        self.enable_sidebar_hover = False
        self.left_sidebar_pinned = False
        self.right_sidebar_pinned = False
        
        # 创建鼠标跟踪区域定时器，用于检测鼠标位置
        self.sidebar_hover_timer = QTimer(self)
        self.sidebar_hover_timer.timeout.connect(self.checkMousePosition)
        self.sidebar_hover_timer.start(50)  # 每50ms检查一次鼠标位置，提高响应速度
        
        # 使用定时器延迟创建悬浮窗口，确保主窗口和RViz框架已完全显示
        # 先创建悬浮窗口，然后再自动隐藏左侧栏，避免冲突
        QTimer.singleShot(1000, self.setupAllOverlaysAndHideSidebar)

        ## 禁用菜单栏、状态栏和"隐藏停靠"按钮
        self.frame.setMenuBar(None)
        self.frame.setStatusBar(None)
        self.frame.setHideButtonVisibility(True)

        ## 获取VisualizationManager实例
        self.manager = self.frame.getManager()
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt(0)
        

        
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
        function_layout.setSpacing(15)  # 增加按钮间距
        
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
        status_layout.setSpacing(15)
        
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
        self.left_sidebar.setFixedWidth(500)  # 设置固定宽度500px
        # 使用QSizePolicy允许垂直方向缩放
        self.left_sidebar.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)
        left_sidebar_layout = QVBoxLayout(self.left_sidebar)
        left_sidebar_layout.setContentsMargins(10, 10, 10, 10)  # 增加边距
        left_sidebar_layout.setSpacing(10)  # 减小组件间距
        
        # 添加无人机状态组件
        status_group = QGroupBox("无人机状态")
        status_group.setStyleSheet("color: #3498DB; font-size: 14pt;")  # 增大字体
        status_group_layout = QVBoxLayout(status_group)
        status_group_layout.setContentsMargins(10, 20, 10, 10)  # 增加内边距
        status_group_layout.setSpacing(15)  # 增加组件间距
        
        # 创建无人机状态信息容器，使用垂直布局使界面更美观
        info_container = QWidget()
        info_container.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)  # 允许扩展
        info_layout = QVBoxLayout(info_container)
        info_layout.setContentsMargins(10, 15, 10, 15)  # 增加内边距使布局更美观
        info_layout.setSpacing(20)  # 增加间距
        
        # 创建两列布局的容器
        status_grid = QWidget()
        status_grid_layout = QGridLayout(status_grid)
        status_grid_layout.setContentsMargins(0, 0, 0, 0)
        status_grid_layout.setSpacing(15)  # 合适的间距
        
        # 减小字体大小以确保文本显示完整
        label_style = "font-size: 12pt; font-weight: normal; color: #FFFFFF;"
        value_style = "font-size: 12pt; font-weight: bold; color: #3498DB;"
        
        # 添加重要状态标签 - 使用两列布局
        row = 0
        
        # 第一行 - 无人机模式和连接状态
        mode_label_desc = QLabel("模式:")  # 缩短标签文字
        mode_label_desc.setStyleSheet(label_style)
        status_grid_layout.addWidget(mode_label_desc, row, 0)
        self.mode_label = QLabel("MANUAL")
        self.mode_label.setStyleSheet(value_style)
        self.mode_label.setMinimumWidth(150)  # 增加最小宽度确保文本显示完整
        status_grid_layout.addWidget(self.mode_label, row, 1)
        
        conn_label_desc = QLabel("连接状态:")
        conn_label_desc.setStyleSheet(label_style)
        status_grid_layout.addWidget(conn_label_desc, row, 2)
        self.connection_label = QLabel("已连接")
        self.connection_label.setStyleSheet("font-size: 12pt; font-weight: bold; color: #2ECC71;")
        status_grid_layout.addWidget(self.connection_label, row, 3)
        
        row += 1
        
        # 第二行 - 飞行高度和地面速度
        alt_label_desc = QLabel("飞行高度:")
        alt_label_desc.setStyleSheet(label_style)
        status_grid_layout.addWidget(alt_label_desc, row, 0)
        self.altitude_label = QLabel("0.0000 m")
        self.altitude_label.setStyleSheet(value_style)
        status_grid_layout.addWidget(self.altitude_label, row, 1)
        
        speed_label_desc = QLabel("地面速度:")
        speed_label_desc.setStyleSheet(label_style)
        status_grid_layout.addWidget(speed_label_desc, row, 2)
        self.ground_speed_label = QLabel("0.0000 m/s")
        self.ground_speed_label.setStyleSheet(value_style)
        status_grid_layout.addWidget(self.ground_speed_label, row, 3)
        
        row += 1
        
        # 第三行 - 姿态角度信息
        pitch_label_desc = QLabel("俯仰角:")
        pitch_label_desc.setStyleSheet(label_style)
        status_grid_layout.addWidget(pitch_label_desc, row, 0)
        self.pitch_label = QLabel("0.00°")
        self.pitch_label.setStyleSheet(value_style)
        status_grid_layout.addWidget(self.pitch_label, row, 1)
        
        roll_label_desc = QLabel("滚转角:")
        roll_label_desc.setStyleSheet(label_style)
        status_grid_layout.addWidget(roll_label_desc, row, 2)
        self.roll_label = QLabel("0.00°")
        self.roll_label.setStyleSheet(value_style)
        status_grid_layout.addWidget(self.roll_label, row, 3)
        
        row += 1
        
        # 第四行 - 偏航角和电池状态
        yaw_label_desc = QLabel("偏航角:")
        yaw_label_desc.setStyleSheet(label_style)
        status_grid_layout.addWidget(yaw_label_desc, row, 0)
        self.yaw_label = QLabel("0.00°")
        self.yaw_label.setStyleSheet(value_style)
        status_grid_layout.addWidget(self.yaw_label, row, 1)
        
        battery_status_desc = QLabel("电池状态:")
        battery_status_desc.setStyleSheet(label_style)
        status_grid_layout.addWidget(battery_status_desc, row, 2)
        self.battery_status_label = QLabel(f"{self.battery_percentage:.1f}% ({self.battery_voltage:.2f}V)")
        self.battery_status_label.setStyleSheet(value_style)
        status_grid_layout.addWidget(self.battery_status_label, row, 3)
        
        # 设置列宽度比例，确保均匀分布
        for col in range(4):
            status_grid_layout.setColumnStretch(col, 1)
            
        # 添加状态网格到容器
        info_layout.addWidget(status_grid)
        
        # 添加美观的分隔线
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setFrameShadow(QFrame.Sunken)
        separator.setStyleSheet("background-color: #3498DB; min-height: 2px;")
        info_layout.addWidget(separator)
        
        status_group_layout.addWidget(info_container, 1)  # 使用拉伸系数1
        
        # 添加状态组到左侧边栏
        left_sidebar_layout.addWidget(status_group, 2)  # 使用更大的拉伸系数
        
        # 添加功能区域组件（与状态区分离）
        function_group = QGroupBox("控制中心")
        function_group.setStyleSheet("color: #3498DB; font-size: 14pt; margin-top: 15px;")  # 设置标题样式并增加顶部边距
        function_group.setTitle("  控制中心  ")  # 通过增加空格让标题文字有更多显示空间
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
            # 设置控件尺寸为合适的大小，保持扇形形状清晰可见
            self.ui_button.setMinimumSize(350, 350)
            self.ui_button.setMaximumSize(660, 660)
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
            function_layout.setSpacing(5)  # 减小组件间距
            
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
        
        # 添加功能组到左侧边栏
        left_sidebar_layout.addWidget(function_group, 1)  # 给控制中心更多空间
        
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
        self.right_sidebar.setFixedWidth(650)  # 设置固定宽度650px
        # 使右侧栏可以在垂直方向调整大小
        self.right_sidebar.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)
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
        person_position_group = QGroupBox("待搜索人员位置")
        person_position_group.setStyleSheet("color: #3498DB; font-size: 14pt;")  # 设置标题样式
        # 设置大小策略为垂直方向可扩展
        person_position_group.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        person_position_layout = QVBoxLayout(person_position_group)
        person_position_layout.setContentsMargins(10, 20, 10, 10)  # 增加内边距
        person_position_layout.setSpacing(10)  # 减少组件间距以节省空间
        
        # 创建位置显示区域
        position_frame = QFrame()
        position_frame.setFrameShape(QFrame.StyledPanel)
        position_frame.setStyleSheet("background-color: #1A202C; border-radius: 10px; border: 1px solid #3498DB;")
        # 设置Frame可扩展
        position_frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        position_frame_layout = QVBoxLayout(position_frame)
        position_frame_layout.setContentsMargins(10, 10, 10, 10)
        
        # 创建位置信息表格
        self.position_table = QTableWidget()
        self.position_table.setColumnCount(4)
        self.position_table.setHorizontalHeaderLabels(["ID", "X坐标", "Y坐标", "状态"])
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
                padding: 5px;
                border: 1px solid #3498DB;
            }
            QTableWidget::item {
                border-bottom: 1px solid #3498DB;
                padding: 5px;
            }
            QTableWidget::item:selected {
                background-color: #3498DB;
            }
        """)
        self.position_table.horizontalHeader().setStretchLastSection(True)
        self.position_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.position_table.verticalHeader().setVisible(False)
        # 设置表格可扩展，但有最小高度限制
        self.position_table.setMinimumHeight(120)
        # 不设置最大高度限制，允许根据可用空间自动调整
        self.position_table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # 将表格初始化为空
        self.position_table.setRowCount(0)
        
        # 添加表格到位置框架
        position_frame_layout.addWidget(self.position_table)
        
        # 添加操作按钮区域
        button_container = QWidget()
        button_layout = QHBoxLayout(button_container)
        button_layout.setContentsMargins(0, 5, 0, 0)
        button_layout.setSpacing(10)
        
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
        
        # 创建鸟瞰图显示区域
        bird_view_container = QWidget()
        bird_view_layout = QVBoxLayout(bird_view_container)
        bird_view_layout.setContentsMargins(0, 0, 0, 0)
        bird_view_layout.setSpacing(0)
        
        # 鸟瞰图标签
        bird_view_title = QLabel("障碍物鸟瞰图")
        bird_view_title.setStyleSheet("font-size: 10pt; font-weight: bold; color: #3498DB; background-color: #2C3E50; padding: 5px;")
        bird_view_title.setAlignment(Qt.AlignCenter)
        bird_view_title.setFixedHeight(30)
        bird_view_layout.addWidget(bird_view_title)
        
        # 鸟瞰图显示
        self.bird_view_label = QLabel()
        self.bird_view_label.setAlignment(Qt.AlignCenter)
        self.bird_view_label.setFixedSize(640, 240)  # 固定尺寸为640x240
        self.bird_view_label.setStyleSheet("background-color: #1A202C; border: 1px solid #3498DB; border-top: none;")
        self.bird_view_label.setText("等待鸟瞰图数据...")
        bird_view_layout.addWidget(self.bird_view_label)
        
        # 添加鸟瞰图容器到图像显示容器
        image_display_layout.addWidget(bird_view_container)
        
        # 添加一个小间隔
        spacer = QWidget()
        spacer.setFixedHeight(5)  # 减小间隔
        image_display_layout.addWidget(spacer)
        
        # 创建按钮区域 - 平行四边形按钮
        button_container = QWidget()
        button_layout = QHBoxLayout(button_container)
        button_layout.setContentsMargins(0, 0, 0, 0)
        button_layout.setSpacing(0)  # 按钮之间无间距
        
        # 设置按钮容器高度
        button_container.setFixedHeight(30)
        
        # RGB图像按钮 - 深色
        self.rgb_button = QPushButton("RGB图像")
        self.rgb_button.setStyleSheet("""
            QPushButton {
                background-color: #2C3E50;  /* 深色 */
                color: white;
                border-radius: 0;  /* 无圆角 */
                border: none;
                font-weight: bold;
                font-size: 10pt;  /* 固定字体大小 */
                padding: 2px;
                min-width: 320px;
                text-align: center;
            }
            QPushButton:hover {
                background-color: #234567;
            }
            QPushButton:checked {
                background-color: #1A202C;  /* 选中时更深的颜色 */
                color: white;
                border-radius: 0;  /* 无圆角 */
                border: none;
                font-weight: bold;
                font-size: 10pt;  /* 固定字体大小 */
                padding: 2px;
                min-width: 320px;
                text-align: center;
            }
        """)
        self.rgb_button.setCheckable(True)
        self.rgb_button.setChecked(True)
        self.rgb_button.clicked.connect(self.switchToRGBImage)
        button_layout.addWidget(self.rgb_button)
        
        # 深度图像按钮 - 浅色
        self.depth_button = QPushButton("深度图像")
        self.depth_button.setStyleSheet("""
            QPushButton {
                background-color: #3498DB;  /* 浅色 */
                color: white;
                border-radius: 0;  /* 无圆角 */
                border: none;
                font-weight: bold;
                font-size: 10pt;  /* 固定字体大小 */
                padding: 2px;
                min-width: 320px;
                text-align: center;
            }
            QPushButton:hover {
                background-color: #2980B9;
            }
            QPushButton:checked {
                background-color: #1A202C;  /* 选中时更深的颜色 */
                color: white;
                border-radius: 0;  /* 无圆角 */
                border: none;
                font-weight: bold;
                font-size: 10pt;  /* 固定字体大小 */
                padding: 2px;
                min-width: 320px;
                text-align: center;
            }
        """)
        self.depth_button.setCheckable(True)
        self.depth_button.clicked.connect(self.switchToDepthImage)
        button_layout.addWidget(self.depth_button)
        
        # 添加按钮容器到图像显示布局
        image_display_layout.addWidget(button_container)

        # 创建图像显示区域
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setFixedSize(640, 480)  # 固定尺寸为640x480
        self.image_label.setStyleSheet("background-color: #1A202C; border: 1px solid #3498DB; border-top: none;")
        self.image_label.setText("等待图像...")
        image_display_layout.addWidget(self.image_label)
        
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
        
        # 设置大分辨率下的分割器初始比例 - 以百分比形式
        total_width = QDesktopWidget().availableGeometry().width()
        if total_width > 1920:  # 对于大分辨率屏幕
            # 假设总宽为100，分配左:中:右 = 20:50:30的比例
            left_width = int(total_width * 0.2)
            right_width = int(total_width * 0.3)
            center_width = total_width - left_width - right_width - 40  # 40是两个控制条的宽度
            self.main_splitter.setSizes([left_width, 20, center_width, 20, right_width])

        # 禁止分割器伸缩右侧栏
        self.main_splitter.setStretchFactor(0, 0)  # 左侧栏不自动拉伸
        self.main_splitter.setStretchFactor(1, 0)  # 左侧控制按钮不自动拉伸
        self.main_splitter.setStretchFactor(2, 1)  # 中间RViz区域自动拉伸
        self.main_splitter.setStretchFactor(3, 0)  # 右侧控制按钮不自动拉伸
        self.main_splitter.setStretchFactor(4, 0)  # 右侧栏不自动拉伸
        
        # 设置初始分割比例
        total_width = self.width()  # 获取窗口总宽度
        remaining_width = total_width - 500 - 20 - 20 - 650  # 总宽度减去左侧栏、左侧控制按钮、右侧控制按钮和右侧栏的宽度
        self.main_splitter.setSizes([500, 20, remaining_width, 20, 650])  # 左侧栏500px，左控制按钮20px，中间RViz区域自适应，右控制按钮20px，右侧栏650px
        
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
        
        # 设置定时器以更新状态栏信息
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.updateStatusBar)
        self.status_timer.start(100)  # 每100ms更新一次
        
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
            self.connection_label.setStyleSheet("color: #E74C3C;")
        if hasattr(self, 'mode_label'):
            self.mode_label.setText("未连接")
        
        # 设置图像更新定时器
        self.image_timer = QTimer(self)
        self.image_timer.timeout.connect(self.updateImageDisplay)
        self.image_timer.start(33)  # 约30fps
        
        main_layout.addWidget(status_bar)
        
        # 设置布局的拉伸因子，让中间的RViz显示区域占据大部分空间
        main_layout.setStretch(1, 10)  # 中间部分(splitter)占据更多空间
        main_layout.setStretch(0, 1)   # 标题栏占据较少空间
        main_layout.setStretch(2, 1)   # 底部栏占据较少空间
        
        self.setLayout(main_layout)
        
        # 程序启动时自动初始化话题订阅器，无需等待点击一键启动按钮
        QTimer.singleShot(2000, self.setupTopicSubscriber)  # 延迟2秒初始化订阅器，确保界面已完全加载
    
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
        else:
            # 显示侧边栏
            if animate:
                # 先设置最大宽度，以便动画可以工作
                self.left_sidebar.setMaximumWidth(500)
                self.left_sidebar.setMinimumWidth(0)
                self.left_sidebar.setVisible(True)
                
                # 使用动画效果
                self.sidebar_animation = QPropertyAnimation(self.left_sidebar, b"maximumWidth")
                self.sidebar_animation.setDuration(200)  # 动画持续时间200ms
                self.sidebar_animation.setStartValue(0)
                self.sidebar_animation.setEndValue(500)
                self.sidebar_animation.setEasingCurve(QEasingCurve.InOutQuad)
                
                # 动画结束后更新状态
                self.sidebar_animation.finished.connect(lambda: self.finishSidebarAnimation(True))
                
                # 启动动画
                self.sidebar_animation.start()
                
                # 立即更新状态
                self.updateSidebarState(True)
                
                # 更新分割器尺寸
                sizes = self.main_splitter.sizes()
                self.main_splitter.setSizes([500, 20, sizes[2] - 500])
            else:
                # 直接显示
                self.left_sidebar.setFixedWidth(500)  # 固定宽度500px
                self.left_sidebar.setVisible(True)
                self.updateSidebarState(True)
                
                # 更新分割器尺寸
                sizes = self.main_splitter.sizes()
                self.main_splitter.setSizes([500, 20, sizes[2] - 500])
    
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
            self.left_sidebar.setMinimumWidth(500)
    
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
                    self.connection_label.setStyleSheet("color: #2ECC71; font-size: 12pt; font-weight: bold;")
                else:
                    self.connection_label.setText("未连接")
                    self.connection_label.setStyleSheet("color: #E74C3C; font-size: 12pt; font-weight: bold;")
            
            # 更新模式显示
            if hasattr(self, 'mode_label'):
                self.mode_label.setText(mode if mode else "UNKNOWN")
            
            # 标记话题有数据
            self.topics_with_data["status"] = True
            
        except Exception as e:
            print(f"更新状态显示时出错: {str(e)}")
    
    def updateCameraImage(self, camera_data):
        """处理摄像头图像更新"""
        try:
            if not camera_data or camera_data["image"] is None:
                return
                
            # 保存最新图像
            self.camera_image = camera_data["image"]
            
        except Exception as e:
            print(f"处理图像更新时出错: {str(e)}")

    def updateImageDisplay(self):
        """更新图像显示"""
        try:
            if self.current_image_mode == "rgb" and self.camera_image is not None:
                # 显示RGB图像
                height, width, channel = self.camera_image.shape
                bytes_per_line = 3 * width
                
                # 将BGR转换为RGB格式
                rgb_image = cv2.cvtColor(self.camera_image, cv2.COLOR_BGR2RGB)
                
                # 创建QImage
                q_image = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
                
                # 创建QPixmap并设置到标签
                pixmap = QPixmap.fromImage(q_image)
                
                # 设置图像到标签，保持宽高比
                self.image_label.setPixmap(pixmap.scaled(
                    640, 
                    480,
                    Qt.KeepAspectRatio, 
                    Qt.SmoothTransformation
                ))
            elif self.current_image_mode == "depth" and self.depth_image is not None:
                # 显示深度图像
                # 规范化深度图像以便可视化
                cv_img = self.depth_image.copy()
                
                # 检查图像类型和通道
                if len(cv_img.shape) == 2:  # 单通道深度图
                    # 归一化到0-255，用于可视化
                    min_val, max_val, _, _ = cv2.minMaxLoc(cv_img)
                    if max_val > min_val:
                        cv_img = cv2.convertScaleAbs(cv_img, alpha=255.0/(max_val-min_val), beta=-min_val*255.0/(max_val-min_val))
                    
                    # 应用彩色映射以便更好地可视化
                    cv_img = cv2.applyColorMap(cv_img, cv2.COLORMAP_JET)
                    
                    # 将BGR转换为RGB格式
                    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
                    
                    height, width, channel = cv_img.shape
                    bytes_per_line = 3 * width
                    
                    # 创建QImage
                    q_image = QImage(cv_img.data, width, height, bytes_per_line, QImage.Format_RGB888)
                else:  # 已经是3通道图像
                    height, width, channel = cv_img.shape
                    bytes_per_line = 3 * width
                    
                    # 将BGR转换为RGB格式
                    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
                    
                    # 创建QImage
                    q_image = QImage(cv_img.data, width, height, bytes_per_line, QImage.Format_RGB888)
                
                # 创建QPixmap并设置到标签
                pixmap = QPixmap.fromImage(q_image)
                
                # 设置图像到标签，保持宽高比
                self.image_label.setPixmap(pixmap.scaled(
                    640, 
                    480,
                    Qt.KeepAspectRatio, 
                    Qt.SmoothTransformation
                ))
            else:
                # 无图像时显示默认文本，使用加大字号的样式使提示更明显
                if hasattr(self, 'image_label') and self.image_label:
                    if not self.topic_subscriber:
                        # 未启动订阅器时显示提示信息
                        self.image_label.setText("<div style='font-size: 16pt; color: #3498DB; text-align: center; margin-top: 200px;'>正在自动连接话题，请稍候...</div>")
                    elif self.current_image_mode == "rgb":
                        if not self.topic_subscriber.is_topic_active("camera"):
                            self.image_label.setText("<div style='font-size: 16pt; color: #3498DB; text-align: center; margin-top: 200px;'>等待RGB图像话题连接...</div>")
                        else:
                            self.image_label.setText("<div style='font-size: 16pt; color: #3498DB; text-align: center; margin-top: 200px;'>等待RGB图像数据...</div>")
                    else:  # depth模式
                        if not self.topic_subscriber.is_topic_active("depth"):
                            self.image_label.setText("<div style='font-size: 16pt; color: #3498DB; text-align: center; margin-top: 200px;'>等待深度图像话题连接...</div>")
                        else:
                            self.image_label.setText("<div style='font-size: 16pt; color: #3498DB; text-align: center; margin-top: 200px;'>等待深度图像数据...</div>")
        except Exception as e:
            print(f"更新图像显示时出错: {str(e)}")
            self.image_label.setText(f"图像显示错误: {str(e)}")
    
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
                
            if hasattr(self, 'dashboard') and self.dashboard:
                self.dashboard.set_speed(0)
                self.dashboard.set_gear(11)  # N挡
                
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
                    self.connection_label.setStyleSheet("color: #E74C3C;")
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
                        self.image_label.setText("<div style='font-size: 16pt; color: #3498DB; text-align: center; margin-top: 200px;'>等待RGB图像话题连接...</div>")
                    # 确保未使用模拟图像
                    self.camera_image = None
                
            # 更新深度图像显示文本 - 使用自定义HTML样式显示
            if hasattr(self, 'image_label'):
                if not self.topic_subscriber.is_topic_active("depth"):
                    if self.current_image_mode == "depth":  # 只在深度模式下更新
                        self.image_label.setText("<div style='font-size: 16pt; color: #3498DB; text-align: center; margin-top: 200px;'>等待深度图像话题连接...</div>")
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
        else:
            # 显示右侧栏
            if animate:
                # 先设置最大宽度，以便动画可以工作
                self.right_sidebar.setMaximumWidth(650)
                self.right_sidebar.setMinimumWidth(0)
                self.right_sidebar.setVisible(True)
                
                # 使用动画效果
                self.right_sidebar_animation = QPropertyAnimation(self.right_sidebar, b"maximumWidth")
                self.right_sidebar_animation.setDuration(200)  # 动画持续时间200ms
                self.right_sidebar_animation.setStartValue(0)
                self.right_sidebar_animation.setEndValue(650)
                self.right_sidebar_animation.setEasingCurve(QEasingCurve.InOutQuad)
                
                # 动画结束后更新状态
                self.right_sidebar_animation.finished.connect(lambda: self.finishRightSidebarAnimation(True))
                
                # 启动动画
                self.right_sidebar_animation.start()
                
                # 立即更新状态
                self.updateRightSidebarState(True)
                
                # 更新分割器尺寸
                sizes = self.main_splitter.sizes()
                if sizes[2] > 650:  # 确保中间区域有足够空间
                    new_sizes = [sizes[0], sizes[1], sizes[2] - 650, sizes[3], 650]
                else:  # 如果中间区域空间不足，则按比例分配
                    total_space = sizes[2]
                    new_middle = max(int(total_space * 0.4), 100)  # 至少保留100px给中间区域
                    new_sizes = [sizes[0], sizes[1], new_middle, sizes[3], total_space - new_middle]
                self.main_splitter.setSizes(new_sizes)
            else:
                # 直接显示
                self.right_sidebar.setFixedWidth(650)  # 固定宽度650px
                self.right_sidebar.setVisible(True)
                self.updateRightSidebarState(True)
                
                # 更新分割器尺寸
                sizes = self.main_splitter.sizes()
                if sizes[2] > 650:  # 确保中间区域有足够空间
                    new_sizes = [sizes[0], sizes[1], sizes[2] - 650, sizes[3], 650]
                else:  # 如果中间区域空间不足，则按比例分配
                    total_space = sizes[2]
                    new_middle = max(int(total_space * 0.4), 100)  # 至少保留100px给中间区域
                    new_sizes = [sizes[0], sizes[1], new_middle, sizes[3], total_space - new_middle]
                self.main_splitter.setSizes(new_sizes)
                
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
            self.right_sidebar.setMinimumWidth(650)
    
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
                    
                    # 更新姿态标签
                    if hasattr(self, 'pitch_label'):
                        self.pitch_label.setText(f"{pitch_value:.2f}°")
                    if hasattr(self, 'roll_label'):
                        self.roll_label.setText(f"{roll_value:.2f}°")
                    if hasattr(self, 'yaw_label'):
                        self.yaw_label.setText(f"{yaw_value:.2f}°")
                    
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
            # 创建日志目录
            # 使用当前目录下的log文件夹而不是/tmp目录
            current_dir = os.path.dirname(os.path.abspath(__file__))
            log_dir = os.path.join(current_dir, "log")
            os.makedirs(log_dir, exist_ok=True)
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
            
            # 等待40秒，确保所有节点启动完成
            timeout = 40  # 增加到40秒等待，与run.sh中的累计睡眠时间一致
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
                    
                # 更新进度条 - 在40秒内从10%逐步增加到50%
                elapsed = time.time() - start_time
                progress = int(10 + min(40, (elapsed / timeout * 40)))
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
            
            # 延迟启动第二个进程 - 给run.sh额外的10秒时间完成启动
            QTimer.singleShot(10000, lambda: self.startSecondProcess(progress_dialog))
            
        except Exception as e:
            QMessageBox.critical(self, "启动错误", f"启动无人机系统时出错: {str(e)}")
            
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
            # 使用当前目录下的log文件夹
            current_dir = os.path.dirname(os.path.abspath(__file__))
            log_dir = os.path.join(current_dir, "log")
            os.makedirs(log_dir, exist_ok=True)
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
            # 使用当前目录下的log文件夹
            current_dir = os.path.dirname(os.path.abspath(__file__))
            log_dir = os.path.join(current_dir, "log")
            os.makedirs(log_dir, exist_ok=True)
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
            

                
            # 创建新的订阅器
            self.topic_subscriber = TopicsSubscriber()
            
            # 注册回调函数
            self.topic_subscriber.register_callback("battery", self.updateBatteryStatus)
            self.topic_subscriber.register_callback("odometry", self.updatePositionDisplay)
            self.topic_subscriber.register_callback("velocity", self.updateVelocityDisplay)
            self.topic_subscriber.register_callback("status", self.updateStatusDisplay)
            self.topic_subscriber.register_callback("camera", self.updateCameraImage)
            self.topic_subscriber.register_callback("depth", self.updateDepthImage)
            self.topic_subscriber.register_callback("bird_view", self.updateBirdViewImage)
            self.topic_subscriber.register_callback("marker", self.marker_callback)
            self.topic_subscriber.register_callback("attitude", self.updateAttitudeDisplay)
            
            # 添加MAVROS话题回调 - 只保留状态话题，其他与普通话题重复
            self.topic_subscriber.register_callback("mavros_state", self.updateStatusDisplay)
            
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
            # 创建日志目录
            current_dir = os.path.dirname(os.path.abspath(__file__))
            log_dir = os.path.join(current_dir, "log")
            os.makedirs(log_dir, exist_ok=True)
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
            progress_dialog.setLabelText("启动标记计算脚本...")
            QApplication.processEvents()
            time.sleep(3)  # 等待3秒

            # 创建标记计算脚本日志文件
            marker_log = f"{log_dir}/marker_jisuan_{timestamp}.log"
            self.log_files["marker_jisuan"] = marker_log
            print(f"标记计算脚本日志文件: {marker_log}")

            # 启动标记计算脚本（后台运行，输出重定向到日志文件）
            cmd4 = f"cd {zyc_fuel_ws}/scripts && source {zyc_fuel_ws}/devel/setup.bash && python3 marker_wenzi_jisuan.py"
            with open(marker_log, 'w') as log_file:
                marker_process = subprocess.Popen(cmd4, shell=True, stdout=log_file, stderr=log_file, executable='/bin/bash')

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
                # 使用当前目录下的log文件夹
                current_dir = os.path.dirname(os.path.abspath(__file__))
                log_dir = os.path.join(current_dir, "log")
                os.makedirs(log_dir, exist_ok=True)
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
        self.rgb_button.setChecked(True)
        self.depth_button.setChecked(False)
        self.current_image_mode = "rgb"
        # 直接调整背景色而不重设整个样式
        self.rgb_button.setStyleSheet("background-color: #1A202C; font-size: 10pt;")
        self.depth_button.setStyleSheet("background-color: #3498DB; font-size: 10pt;")
        # 更新显示
        self.updateImageDisplay()
    
    def switchToDepthImage(self):
        """切换到深度图像模式"""
        self.rgb_button.setChecked(False)
        self.depth_button.setChecked(True)
        self.current_image_mode = "depth"
        # 直接调整背景色而不重设整个样式
        self.depth_button.setStyleSheet("background-color: #1A202C; font-size: 10pt;")
        self.rgb_button.setStyleSheet("background-color: #3498DB; font-size: 10pt;")
        # 更新显示
        self.updateImageDisplay()
    
    def updateDepthImage(self, depth_data):
        """处理深度图像更新"""
        try:
            if not depth_data or depth_data["image"] is None:
                return
                
            # 保存最新深度图像
            self.depth_image = depth_data["image"]
            
            # 如果当前是深度图像模式，立即更新显示
            if self.current_image_mode == "depth" and hasattr(self, 'image_label'):
                self.updateImageDisplay()
                
        except Exception as e:
            print(f"处理深度图像更新时出错: {str(e)}")

    def updateBirdViewImage(self, bird_view_data):
        """处理鸟瞰图更新"""
        try:
            if not bird_view_data or bird_view_data["image"] is None:
                return
                
            # 保存最新鸟瞰图像
            self.bird_view_image = bird_view_data["image"]
                
            # 输出图像信息
            self.updateBirdViewDisplay()
                
        except Exception as e:
            import traceback
            print(f"处理鸟瞰图更新时出错: {str(e)}")
            print(traceback.format_exc())
    
    def updateBirdViewDisplay(self):
        """更新鸟瞰图显示"""
        try:
            if self.bird_view_image is not None:
                # 检查图像是否为空
                if self.bird_view_image.size == 0 or self.bird_view_image is None:
                    self.bird_view_label.setText("<div style='font-size: 14pt; color: #3498DB; text-align: center; margin-top: 100px;'>收到空图像数据</div>")
                    return
                    
                # 将OpenCV图像转换为Qt图像
                height, width = self.bird_view_image.shape[:2]
                
                # 检查图像类型和通道
                if len(self.bird_view_image.shape) == 3:                   
                    # 彩色图像 - BGR格式
                    bytes_per_line = 3 * width   
                    try:
                        # 将BGR转换为RGB格式
                        rgb_image = cv2.cvtColor(self.bird_view_image, cv2.COLOR_BGR2RGB)
                        
                        # 创建QImage
                        q_image = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
                    except Exception as e:
                        print(f"彩色图像转换失败: {str(e)}")
                        # 如果转换失败，显示错误信息而不是尝试使用原始数据
                        self.bird_view_label.setText(f"<div style='font-size: 12pt; color: #E74C3C; text-align: center; margin-top: 100px;'>图像处理错误:<br>{str(e)}</div>")
                        return
                else:
                    # 灰度图像处理
                    try:
                        # 将灰度图像转换为彩色以便更好的可视化
                        colorized = cv2.cvtColor(self.bird_view_image, cv2.COLOR_GRAY2RGB)
                        bytes_per_line = 3 * width
                        
                        # 创建QImage
                        q_image = QImage(colorized.data, width, height, bytes_per_line, QImage.Format_RGB888)
                    except Exception as e:
                        print(f"灰度图像转换失败: {str(e)}")
                        # 如果转换失败，显示错误信息
                        self.bird_view_label.setText(f"<div style='font-size: 12pt; color: #E74C3C; text-align: center; margin-top: 100px;'>灰度图像处理错误:<br>{str(e)}</div>")
                        return
                
                # 创建QPixmap并设置到标签
                try:
                    pixmap = QPixmap.fromImage(q_image)
                    
                    # 检查pixmap是否为空
                    if pixmap.isNull():
                        self.bird_view_label.setText("<div style='font-size: 12pt; color: #E74C3C; text-align: center; margin-top: 100px;'>无法创建有效图像</div>")
                        return
                        
                    # 设置图像到标签，保持宽高比
                    self.bird_view_label.setPixmap(pixmap.scaled(
                        640, 
                        240,
                        Qt.KeepAspectRatio, 
                        Qt.SmoothTransformation
                    ))
                except Exception as e:
                    print(f"设置鸟瞰图像到UI失败: {str(e)}")
                    self.bird_view_label.setText(f"<div style='font-size: 12pt; color: #E74C3C; text-align: center; margin-top: 100px;'>图像显示错误:<br>{str(e)}</div>")
            else:
                # 无图像时显示默认文本
                if hasattr(self, 'bird_view_label') and self.bird_view_label:
                    if not self.topic_subscriber:
                        # 未启动订阅器时显示提示信息
                        self.bird_view_label.setText("<div style='font-size: 14pt; color: #3498DB; text-align: center; margin-top: 100px;'>正在自动连接话题，请稍候...</div>")
                    else:
                        topic_active = self.topic_subscriber.is_topic_active("bird_view")
                        # 移除调试打印，减少控制台输出
                        if not topic_active:
                            self.bird_view_label.setText("<div style='font-size: 14pt; color: #3498DB; text-align: center; margin-top: 100px;'>等待鸟瞰图话题连接...</div>")
                        else:
                            self.bird_view_label.setText("<div style='font-size: 14pt; color: #3498DB; text-align: center; margin-top: 100px;'>等待鸟瞰图数据...</div>")
        except Exception as e:
            # 捕获所有异常并显示友好的错误信息
            try:
                self.bird_view_label.setText(f"<div style='font-size: 12pt; color: #E74C3C; text-align: center; margin-top: 100px;'>鸟瞰图显示错误:<br>{str(e)}</div>")
            except:
                pass  # 如果连错误信息都无法显示，就不做任何操作
            print(f"鸟瞰图显示更新错误: {str(e)}")
    
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
            
            # 根据窗口大小动态调整布局
            if window_height < 800:
                # 调整鸟瞰图尺寸
                if hasattr(self, 'bird_view_label'):
                    new_height = max(120, int(window_height * 0.15))  # 最小120px
                    self.bird_view_label.setFixedHeight(new_height)
                
                # 调整图像标签尺寸
                if hasattr(self, 'image_label'):
                    new_height = max(240, int(window_height * 0.3))  # 最小240px
                    self.image_label.setFixedHeight(new_height)
                
                # 小窗口模式下简化功能组标题，避免截断
                if hasattr(self, 'function_group') and window_width < 1600:
                    self.function_group.setTitle(" 控制中心 ")
            else:
                # 恢复默认尺寸
                if hasattr(self, 'bird_view_label'):
                    self.bird_view_label.setFixedSize(640, 240)
                
                if hasattr(self, 'image_label'):
                    self.image_label.setFixedSize(640, 480)
                
                # 大窗口模式下扩展功能组标题
                if hasattr(self, 'function_group') and window_width >= 1600:
                    self.function_group.setTitle("   控制中心   ")
            
            # 调用原始的resizeEvent
            QMainWindow.resizeEvent(self, event)
        except Exception as e:
            print(f"调整窗口大小时出错: {str(e)}")
            # 确保原始事件被处理
            QMainWindow.resizeEvent(self, event)
    
    def updateCameraImage(self, camera_data):
        """处理摄像头图像更新"""
        try:
            if not camera_data or camera_data["image"] is None:
                return
                
            # 保存最新图像
            self.camera_image = camera_data["image"]
            
        except Exception as e:
            print(f"处理图像更新时出错: {str(e)}")

    def marker_callback(self, marker_data):
        """处理visualization_marker话题的回调函数"""
        try:
            # 提取标记ID并检查是否已添加过
            marker_id = marker_data["id"]
            if marker_id % 2 == 0:  # 球体标记的ID是偶数
                ball_id = marker_id // 2  # 获取实际的球体ID
                
                # 检查是否已添加过该标记
                if ball_id not in self.detected_markers:
                    # 获取小球坐标
                    x = marker_data["pose"]["position"]["x"]
                    y = marker_data["pose"]["position"]["y"]
                    z = marker_data["pose"]["position"]["z"]
                    
                    # 添加到表格中
                    self._add_marker_to_table(ball_id, x, y, z)
                    
                    # 标记为已添加
                    self.detected_markers.add(ball_id)
                    
                    print(f"检测到新的标记点: ID={ball_id}, 位置: x={x:.2f}, y={y:.2f}, z={z:.2f}")
        except Exception as e:
            print(f"处理标记点数据时出错: {str(e)}")

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
            
            print(f"已添加标记点到表格: ID={ball_id}, X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
            
            # 自动滚动到新添加的行
            self.position_table.scrollToItem(self.position_table.item(row_position, 0))
        except Exception as e:
            print(f"添加标记点到表格时出错: {str(e)}")

    def stopDroneSystem(self):
        """停止无人机系统"""
        try:
            # 确认弹窗
            reply = QMessageBox.question(self, "确认停止", "确定要停止所有无人机系统进程吗？",
                                      QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            
            if reply == QMessageBox.No:
                return
                
            # 立即停止所有进程监控定时器，避免重复弹出错误消息
            for timer_attr in ['check_process_timer', 'check_process2_timer', 'check_process3_timer']:
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
            
            # 进度条继续
            progress_dialog.setValue(38)
            QApplication.processEvents()

            # 使用已存在的停止脚本
            script_path = "/home/togan/zyc_fuel_ws/scripts/stop.sh"
            progress_dialog.setLabelText("正在终止ROS节点(保留roscore)...")
            progress_dialog.setValue(40)
            QApplication.processEvents()
            
            # 执行脚本并实时更新进度
            try:
                # 修改环境变量，通知脚本保留roscore运行
                env = os.environ.copy()
                env["PRESERVE_ROSCORE"] = "1"  # 设置环境变量，告知脚本保留roscore
                
                # 使用带进度展示的方式执行脚本
                process = subprocess.Popen(['bash', script_path], 
                                          stdout=subprocess.PIPE, 
                                          stderr=subprocess.PIPE,
                                          text=True,  # 确保输出是文本格式
                                          bufsize=1,  # 行缓冲
                                          env=env)   # 传递修改后的环境变量
                
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
                
                # 重置图像显示区域状态
                if hasattr(self, 'image_label'):
                    self.image_label.setText("<div style='font-size: 16pt; color: #3498DB; text-align: center; margin-top: 200px;'>系统已停止，请点击\"一键启动\"启动后台程序</div>")
                
                if hasattr(self, 'bird_view_label'):
                    self.bird_view_label.setText("<div style='font-size: 14pt; color: #3498DB; text-align: center; margin-top: 100px;'>系统已停止</div>")
                
                # 清除图像数据
                self.camera_image = None
                self.depth_image = None
                self.bird_view_image = None
                
            except Exception as e:
                if 'progress_dialog' in locals() and progress_dialog is not None:
                    try:
                        progress_dialog.close()
                    except:
                        pass
                error_msg = f"执行停止脚本时出错: {str(e)}"
                QMessageBox.critical(self, "停止失败", error_msg)
                
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
        overlay_layout.setContentsMargins(15, 6, 15, 6)
        overlay_layout.setSpacing(10)  # 增加间距
        
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
            # 如果不是第一项，添加小间距，但不添加分隔线（保持透明效果）
            if i > 0:
                spacer = QSpacerItem(5, 10, QSizePolicy.Fixed, QSizePolicy.Minimum)
                overlay_layout.addItem(spacer)
            
            # 创建一个水平布局的容器来放置图标和值
            item_container = QWidget()
            item_layout = QHBoxLayout(item_container)
            item_layout.setContentsMargins(0, 0, 0, 0)
            item_layout.setSpacing(5)
            
            # 图标
            icon_label = QLabel()
            icon_pixmap = QPixmap(item["icon"]).scaled(22, 22, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            icon_label.setPixmap(icon_pixmap)
            icon_label.setAlignment(Qt.AlignCenter)
            item_layout.addWidget(icon_label)
            
            # 值标签
            value_label = QLabel("--")
            value_label.setProperty("class", "value")
            setattr(self, item["value_id"], value_label)
            item_layout.addWidget(value_label)
            
            # 单位 - 减小与值的间距
            if "unit" in item and item["unit"]:
                unit_label = QLabel(item["unit"])
                unit_label.setContentsMargins(0, 0, 0, 0)  # 移除内边距
                item_layout.setSpacing(2)  # 减小间距
                item_layout.addWidget(unit_label)
            
            overlay_layout.addWidget(item_container)
        
        # 设置固定高度，宽度为RViz宽度的80%
        self.rviz_overlay.setFixedHeight(40)
        
        # 更新位置的函数
        def updateOverlayPosition():
            if hasattr(self, 'frame') and self.frame:
                frame_rect = self.frame.geometry()
                frame_pos = self.frame.mapToGlobal(QPoint(0, 0))
                
                new_width = int(frame_rect.width() * 0.8)
                self.rviz_overlay.setFixedWidth(new_width)
                
                # 居中显示在上方
                x_pos = frame_pos.x() + (frame_rect.width() - new_width) // 2
                y_pos = frame_pos.y() + 20
                
                # 移动窗口
                self.rviz_overlay.move(x_pos, y_pos)
                
                # 确保窗口可见
                if not self.rviz_overlay.isVisible():
                    self.rviz_overlay.show()
        
        # 创建窗口移动和大小变化事件处理函数
        self.frame_move_timer = QTimer(self)
        self.frame_move_timer.timeout.connect(updateOverlayPosition)
        self.frame_move_timer.start(50)  # 50毫秒更新一次位置
        
        # 初始位置更新
        QTimer.singleShot(100, updateOverlayPosition)
        
        # 更新数据的定时器
        self.data_update_timer = QTimer(self)
        self.data_update_timer.timeout.connect(self.updateOverlayData)
        self.data_update_timer.start(300)  # 降低更新频率到300ms

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
                    self.altitude_value.setText(height_value)
                else:
                    self.altitude_value.setText(height_text)
            
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
                    self.speed_value.setText(speed_value)
                else:
                    self.speed_value.setText(speed_text)
            
            # 更新电池电量
            if hasattr(self, 'battery_percentage'):
                battery = f"{self.battery_percentage:.1f}" if isinstance(self.battery_percentage, (int, float)) else "--"
                self.battery_value.setText(battery)
                
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
                    
                # 找到电池图标并更新 - 使用简单方法
                for widget in self.rviz_overlay.findChildren(QLabel):
                    pixmap = widget.pixmap()
                    if pixmap and pixmap.width() == 22 and pixmap.height() == 22:
                        # 检查是否为电池图标（近似判断）
                        if "battery" in widget.objectName() or "battery" in str(widget.property("icon_type")):
                            widget.setPixmap(QPixmap(icon_path).scaled(22, 22, Qt.KeepAspectRatio, Qt.SmoothTransformation))
                            break
            
            # 更新电压
            if hasattr(self, 'battery_voltage'):
                voltage = f"{self.battery_voltage:.2f}" if isinstance(self.battery_voltage, (int, float)) else "--"
                self.voltage_value.setText(voltage)
            
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
            for timer_attr in ['check_process_timer', 'check_process2_timer', 'check_process3_timer']:
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
                    print("已关闭话题订阅器")
                except Exception as e:
                    print(f"关闭话题订阅器时出错: {str(e)}")
            


            # 使用已存在的停止脚本
            script_path = "/home/togan/zyc_fuel_ws/scripts/stop.sh"
            
            # 检查脚本是否存在
            if os.path.exists(script_path):
                try:
                    # 修改环境变量，通知脚本保留roscore运行
                    env = os.environ.copy()
                    env["PRESERVE_ROSCORE"] = "1"  # 设置环境变量，告知脚本保留roscore
                    
                    # 静默执行脚本
                    subprocess.Popen(['bash', script_path], 
                                  stdout=subprocess.PIPE, 
                                  stderr=subprocess.PIPE,
                                  env=env)   # 传递修改后的环境变量
                    
                    print("已启动静默终止进程")
                except Exception as e:
                    print(f"执行停止脚本时出错: {str(e)}")
            else:
                print(f"未找到停止脚本: {script_path}")
                
                # 如果找不到脚本，直接尝试终止常见的ROS节点
                try:
                    # 获取所有节点列表
                    nodes_process = subprocess.run(['rosnode', 'list'], 
                                                stdout=subprocess.PIPE, 
                                                stderr=subprocess.PIPE, 
                                                text=True, 
                                                timeout=5)
                    
                    if nodes_process.returncode == 0:
                        nodes = nodes_process.stdout.strip().split('\n')
                        # 过滤掉rosout和myviz节点（自身）
                        nodes_to_kill = [node for node in nodes if node != '/rosout' and 'myviz' not in node]
                        
                        if nodes_to_kill:
                            kill_cmd = ['rosnode', 'kill'] + nodes_to_kill
                            subprocess.run(kill_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
                            print(f"已尝试终止 {len(nodes_to_kill)} 个ROS节点")
                except Exception as e:
                    print(f"手动终止ROS节点时出错: {str(e)}")
                
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
            if hasattr(self, 'frame') and self.frame:
                frame_rect = self.frame.geometry()
                frame_pos = self.frame.mapToGlobal(QPoint(0, 0))
                
                # 放在右下角，增加边距让它更靠左上方一些
                margin_x = 60  # 增加水平边距
                margin_y = 50  # 增加垂直边距
                x_pos = frame_pos.x() + frame_rect.width() - self.compass.width() - margin_x
                y_pos = frame_pos.y() + frame_rect.height() - self.compass.height() - margin_y
                
                # 移动窗口
                self.compass.move(x_pos, y_pos)
                
                # 确保窗口可见
                if not self.compass.isVisible():
                    self.compass.show()
        
        # 将函数保存为类实例方法，以便后续修改
        self.updateCompassPosition = updateCompassPosition
        
        # 创建窗口移动和大小变化事件处理函数
        self.compass_move_timer = QTimer(self)
        self.compass_move_timer.timeout.connect(updateCompassPosition)
        self.compass_move_timer.start(50)  # 50毫秒更新一次位置
        
        # 初始位置更新
        QTimer.singleShot(100, updateCompassPosition)
        
        # 创建更新数据的函数
        def updateCompassData():
            try:
                if hasattr(self, 'topic_subscriber') and self.topic_subscriber:
                    # 尝试从姿态数据获取航向信息
                    attitude_data = self.topic_subscriber.get_latest_data("attitude")
                    if attitude_data and "yaw" in attitude_data:
                        # 检查yaw是否为列表，如果是则取第一个元素
                        yaw_value = attitude_data["yaw"]
                        if isinstance(yaw_value, list):
                            if len(yaw_value) > 0:
                                yaw_value = yaw_value[0]  # 取列表的第一个元素
                            else:
                                yaw_value = 0  # 空列表时使用默认值
                        
                        # 将弧度转换为角度并取模360
                        heading = math.degrees(yaw_value) % 360
                        self.compass.set_heading(heading)
                    
                    # 如果没有姿态数据，也可以尝试从GPS航向获取
                    if (not attitude_data or "yaw" not in attitude_data) and hasattr(self.topic_subscriber, 'get_latest_data'):
                        gps_data = self.topic_subscriber.get_latest_data("gps")
                        if gps_data and "heading" in gps_data:
                            heading_value = gps_data["heading"]
                            # 同样检查heading是否为列表
                            if isinstance(heading_value, list):
                                if len(heading_value) > 0:
                                    heading_value = heading_value[0]
                                else:
                                    heading_value = 0
                            
                            heading = heading_value % 360
                            self.compass.set_heading(heading)
            except Exception as e:
                print(f"更新指南针数据时出错: {str(e)}")
        
        # 创建定时更新的定时器
        self.compass_update_timer = QTimer(self)
        self.compass_update_timer.timeout.connect(updateCompassData)
        self.compass_update_timer.start(100)  # 10Hz更新频率

    def setupAttitudeWidget(self):
        """创建姿态指示器组件并添加到RViz右下角，独立窗口但跟随RViz框架移动"""
        # 导入姿态指示器组件
        from dashboard import AttitudeIndicatorWidget
        
        # 创建姿态指示器组件
        self.attitude_widget = AttitudeIndicatorWidget(parent=self)  # 传入self作为参考，但不作为父组件
        
        # 定义位置更新函数 - 直接基于RViz框架位置计算
        def updateAttitudeWidgetPosition():
            if hasattr(self, 'frame') and self.frame:
                frame_rect = self.frame.geometry()
                frame_pos = self.frame.mapToGlobal(QPoint(0, 0))
                
                # 放在右下角位置，与指南针并排
                margin_x = 260  # 增加水平边距，放在指南针左侧
                margin_y = 50   # 垂直边距与指南针相同
                x_pos = frame_pos.x() + frame_rect.width() - self.attitude_widget.width() - margin_x
                y_pos = frame_pos.y() + frame_rect.height() - self.attitude_widget.height() - margin_y
                
                # 确保不会移出窗口左侧
                if x_pos < frame_pos.x() + 20:
                    # 如果左侧空间不足，放在上方
                    x_pos = frame_pos.x() + frame_rect.width() - self.attitude_widget.width() - 60
                    y_pos = frame_pos.y() + frame_rect.height() - self.attitude_widget.height() - 240
                
                # 移动窗口
                self.attitude_widget.move(x_pos, y_pos)
                
                # 确保窗口可见
                if not self.attitude_widget.isVisible():
                    self.attitude_widget.show()
        
        # 将函数保存为类实例方法，以便后续修改
        self.updateAttitudeWidgetPosition = updateAttitudeWidgetPosition
        
        # 创建窗口移动和大小变化事件处理函数
        self.attitude_move_timer = QTimer(self)
        self.attitude_move_timer.timeout.connect(updateAttitudeWidgetPosition)
        self.attitude_move_timer.start(50)  # 50毫秒更新一次位置
        
        # 初始位置更新
        QTimer.singleShot(100, updateAttitudeWidgetPosition)
        
        # 创建更新数据的函数
        def updateAttitudeWidgetData():
            if hasattr(self, 'topic_subscriber') and self.topic_subscriber:
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
        
        # 创建定时更新的定时器
        self.attitude_widget_update_timer = QTimer(self)
        self.attitude_widget_update_timer.timeout.connect(updateAttitudeWidgetData)
        self.attitude_widget_update_timer.start(50)  # 20Hz更新频率

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
        
    def handleCloseEvent(self, event):
        """关闭事件处理函数，确保关闭时清理资源"""
        # 关闭所有悬浮窗口
        if hasattr(self, 'rviz_overlay') and self.rviz_overlay:
            self.rviz_overlay.close()
        
        if hasattr(self, 'compass') and self.compass:
            self.compass.close()
            
        if hasattr(self, 'attitude_widget') and self.attitude_widget:
            self.attitude_widget.close()
            
        # 停止所有定时器
        if hasattr(self, 'frame_move_timer') and self.frame_move_timer:
            self.frame_move_timer.stop()
            
        if hasattr(self, 'compass_move_timer') and self.compass_move_timer:
            self.compass_move_timer.stop()
            
        if hasattr(self, 'attitude_move_timer') and self.attitude_move_timer:
            self.attitude_move_timer.stop()
            
        # 静默关闭后台程序（不显示任何对话框）
        try:
            self.silentStopDroneSystem()
        except Exception as e:
            print(f"静默关闭程序时出错: {str(e)}")
            
        # 接受关闭事件
        event.accept()

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
    
    # 确保应用程序支持中文
    QTextCodec.setCodecForLocale(QTextCodec.codecForName("UTF-8"))
    
    # 检查并自动启动roscore
    check_and_start_roscore()
    
    # 初始化ROS节点
    try:
        rospy.init_node('myviz', anonymous=True)
        print("成功初始化ROS节点: myviz")
    except Exception as e:
        print(f"警告: ROS节点初始化失败: {str(e)}")
    
    # 创建主窗口
    myviz = MyViz()
    
    # 不要使用自定义布局，避免"already has a layout"错误
    # myviz.setLayout(main_layout) # 删除这一行

    # 获取可用屏幕区域（考虑任务栏/面板）
    desktop = QDesktopWidget()
    available_geometry = desktop.availableGeometry(desktop.primaryScreen())
    width = available_geometry.width()
    height = available_geometry.height()
    print(f"可用屏幕区域: {width}x{height}, 位置: ({available_geometry.x()}, {available_geometry.y()})")
    
    # 直接以最大化模式启动窗口
    myviz.showMaximized()
    
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