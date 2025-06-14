#!/usr/bin/env python
# -*- coding: utf-8 -*-
## First we start with the standard ros Python import line:
import roslib; roslib.load_manifest('rviz_python_tutorial')
import rospy

## Then load sys to get sys.argv.
import sys
import time
import os
import json
import random
import math
import numpy as np
import cv2

## 导入我们创建的话题订阅模块、仪表盘组件和话题日志组件
try:
    from topics_subscriber import TopicsSubscriber
except ImportError:
    print("无法导入topics_subscriber模块")
    TopicsSubscriber = None

try:
    from dashboard import DashBoard
except ImportError:
    print("无法导入dashboard模块")
    DashBoard = None
    
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
        
        # 存储已检测到的标记点ID，避免重复添加
        self.detected_markers = set()
        
        # 设置窗口标题
        self.setWindowTitle("无人机自主搜救系统")
        
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
        
        ## rviz.VisualizationFrame是RViz应用程序的主容器窗口小部件
        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath("")
        self.frame.initialize()

        ## 读取配置文件
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile(config, "my_config.rviz")
        self.frame.load(config)
        
        # 初始化日志窗口
        self.log_window = None

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
        title_label = QLabel("无人机自主搜救系统")
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
        
        # 创建速度表盘
        if DashBoard:
            dashboard_container = QWidget()
            dashboard_container.setMinimumHeight(320)  # 增加最小高度
            dashboard_container.setMinimumWidth(320)   # 设置最小宽度
            dashboard_container.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)  # 允许扩展
            dashboard_layout = QVBoxLayout(dashboard_container)
            dashboard_layout.setContentsMargins(0, 0, 0, 0)
            
            self.dashboard = DashBoard(dashboard_container)
            dashboard_layout.addWidget(self.dashboard)
            
            # 添加框架突出显示仪表盘
            dashboard_frame = QFrame()
            dashboard_frame.setFrameShape(QFrame.StyledPanel)
            dashboard_frame.setStyleSheet("background-color: #1A202C; border-radius: 10px; border: 1px solid #3498DB;")
            dashboard_frame_layout = QVBoxLayout(dashboard_frame)
            dashboard_frame_layout.setContentsMargins(5, 5, 5, 5)
            dashboard_frame_layout.addWidget(dashboard_container)
            
            status_group_layout.addWidget(dashboard_frame, 1)  # 给仪表盘更多空间
        else:
            # 如果仪表盘模块不可用，显示错误信息
            error_label = QLabel("速度表盘组件不可用")
            error_label.setStyleSheet("color: red;")
            error_label.setAlignment(Qt.AlignCenter)
            status_group_layout.addWidget(error_label)
        
        # 添加其他状态信息
        info_container = QWidget()
        info_layout = QGridLayout(info_container)
        info_layout.setContentsMargins(5, 5, 5, 5)
        info_layout.setSpacing(15)  # 增加间距
        
        # 添加各种状态标签
        # 增加标签字体大小
        label_style = "font-size: 12pt; font-weight: normal; color: #FFFFFF;"
        value_style = "font-size: 12pt; font-weight: bold; color: #3498DB;"
        
        mode_label_desc = QLabel("无人机模式:")
        mode_label_desc.setStyleSheet(label_style)
        info_layout.addWidget(mode_label_desc, 0, 0)
        self.mode_label = QLabel("MANUAL")
        self.mode_label.setStyleSheet(value_style)
        info_layout.addWidget(self.mode_label, 0, 1)
        
        conn_label_desc = QLabel("连接状态:")
        conn_label_desc.setStyleSheet(label_style)
        info_layout.addWidget(conn_label_desc, 1, 0)
        self.connection_label = QLabel("已连接")
        self.connection_label.setStyleSheet("font-size: 12pt; font-weight: bold; color: #2ECC71;")
        info_layout.addWidget(self.connection_label, 1, 1)
        
        alt_label_desc = QLabel("飞行高度:")
        alt_label_desc.setStyleSheet(label_style)
        info_layout.addWidget(alt_label_desc, 2, 0)
        self.altitude_label = QLabel("0.0000 m")
        self.altitude_label.setStyleSheet(value_style)
        info_layout.addWidget(self.altitude_label, 2, 1)
        
        speed_label_desc = QLabel("地面速度:")
        speed_label_desc.setStyleSheet(label_style)
        info_layout.addWidget(speed_label_desc, 3, 0)
        self.ground_speed_label = QLabel("0.0000 m/s")
        self.ground_speed_label.setStyleSheet(value_style)
        info_layout.addWidget(self.ground_speed_label, 3, 1)
        
        status_group_layout.addWidget(info_container)
        
        # 添加状态组到左侧边栏
        left_sidebar_layout.addWidget(status_group)
        
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
        
        function_layout.addWidget(start_btn, 1, 1)
        
        # 创建右侧按钮 - 待开发 - 文字竖向排列
        future_btn_right = QPushButton()
        future_btn_right.setCursor(Qt.PointingHandCursor)  # 设置鼠标悬停时的光标为手型
        future_btn_right.setStyleSheet("""
            QPushButton {
                background-color: #7F8C8D;
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
                background-color: #95A5A6;
            }
            QPushButton:pressed {
                background-color: #707B7C;
            }
        """)
        # 添加阴影效果
        shadow4 = QGraphicsDropShadowEffect()
        shadow4.setBlurRadius(10)
        shadow4.setColor(QColor(0, 0, 0, 60))
        shadow4.setOffset(2, 2)
        future_btn_right.setGraphicsEffect(shadow4)
        # 创建竖向文字标签
        future_label = QLabel("待\n开\n发")
        future_label.setAlignment(Qt.AlignCenter)
        future_label.setStyleSheet("color: white; background-color: transparent; font-size: 12pt; font-weight: bold;")
        # 添加标签到按钮
        future_layout = QVBoxLayout(future_btn_right)
        future_layout.setContentsMargins(5, 5, 5, 5)
        future_layout.addWidget(future_label, 0, Qt.AlignCenter)
        
        function_layout.addWidget(future_btn_right, 1, 2)
        
        # 创建底部按钮 - 待开发
        future_btn_bottom = QPushButton("待开发")
        future_btn_bottom.setCursor(Qt.PointingHandCursor)  # 设置鼠标悬停时的光标为手型
        future_btn_bottom.setStyleSheet("""
            QPushButton {
                background-color: #7F8C8D;
                color: white;
                border-radius: 8px;
                font-size: 12pt;
                font-weight: bold;
                padding: 8px;
                min-height: 40px;
                border: none;
            }
            QPushButton:hover {
                background-color: #95A5A6;
            }
            QPushButton:pressed {
                background-color: #707B7C;
            }
        """)
        # 添加阴影效果
        shadow5 = QGraphicsDropShadowEffect()
        shadow5.setBlurRadius(10)
        shadow5.setColor(QColor(0, 0, 0, 60))
        shadow5.setOffset(2, 2)
        future_btn_bottom.setGraphicsEffect(shadow5)
        
        function_layout.addWidget(future_btn_bottom, 2, 1)
        
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
        function_layout.setAlignment(future_btn_bottom, Qt.AlignCenter)
        
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
        
        # 当按钮被点击时触发侧边栏的显示/隐藏
        self.sidebar_expanded = True
        self.toggle_sidebar_btn.clicked.connect(self.toggleSidebar)
        
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
        
        # 当按钮被点击时触发右侧栏的显示/隐藏
        self.right_sidebar_expanded = True
        self.toggle_right_sidebar_btn.clicked.connect(self.toggleRightSidebar)
        
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
        
        # 添加待搜救人员位置窗口
        person_position_group = QGroupBox("待搜救人员位置")
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
        self.position_label.setStyleSheet("color: #3498DB; padding-left: 15px;")
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
        self.ros_time_label.setStyleSheet("color: #3498DB;")
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
        self.gear = 4  # 默认挡位
        
        # 初始化话题订阅器(将在后台自动连接话题)
        try:
            self.topic_subscriber = TopicsSubscriber()
            # 注册电池状态更新回调
            self.topic_subscriber.register_callback("battery", self.updateBatteryStatus)
            self.topic_subscriber.register_callback("odometry", self.updatePositionDisplay)
            self.topic_subscriber.register_callback("velocity", self.updateVelocityDisplay)
            self.topic_subscriber.register_callback("status", self.updateStatusDisplay)
            self.topic_subscriber.register_callback("camera", self.updateCameraImage)
            self.topic_subscriber.register_callback("depth", self.updateDepthImage)
            self.topic_subscriber.register_callback("bird_view", self.updateBirdViewImage)
            self.topic_subscriber.register_callback("marker", self.marker_callback)
            print("话题订阅器已启动，将在后台自动连接可用话题...")
            
            # 初始状态设置为未连接
            if hasattr(self, 'connection_label'):
                self.connection_label.setText("未连接")
                self.connection_label.setStyleSheet("color: #E74C3C;")
            if hasattr(self, 'mode_label'):
                self.mode_label.setText("未连接")
                
        except Exception as e:
            print(f"初始化话题订阅器失败: {str(e)}")
            self.topic_subscriber = None
        
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
    
    def toggleDisplayPanel(self):
        """此方法已不再使用，保留以避免可能的引用错误"""
        self.toggleRVizDisplayPanel()
    
    def toggleSidebar(self):
        """显示或隐藏侧边栏"""
        if self.sidebar_expanded:
            # 隐藏侧边栏
            self.left_sidebar.setMaximumWidth(0)
            self.left_sidebar.setMinimumWidth(0)
            self.toggle_sidebar_btn.setIcon(QIcon(":/images/icons/dropright.svg"))  # 切换图标为右箭头
            self.sidebar_expanded = False
            
            # 更新分割器尺寸
            sizes = self.main_splitter.sizes()
            self.main_splitter.setSizes([0, 20, sizes[0] + sizes[2]])
        else:
            # 恢复侧边栏
            self.left_sidebar.setFixedWidth(500)  # 固定宽度500px
            self.toggle_sidebar_btn.setIcon(QIcon(":/images/icons/dropleft.svg"))  # 切换图标为左箭头
            self.sidebar_expanded = True
            
            # 更新分割器尺寸
            sizes = self.main_splitter.sizes()
            self.main_splitter.setSizes([500, 20, sizes[2] - 500])

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
            if not battery_data or not hasattr(self, 'voltage_label') or not hasattr(self, 'battery_icon_label'):
                return
                
            # 更新电池百分比和电压
            percentage = battery_data.get("percentage", 0.0) * 100  # 转换为百分比
            voltage = battery_data.get("voltage", 0.0)  # 获取电压值
            
            # 保存数据以便在模拟模式下使用
            self.battery_percentage = percentage
            self.battery_voltage = voltage
            
            # 更新电压显示
            self.voltage_label.setText(f"{voltage:.2f} V")
            
            # 根据电量选择对应图标
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
        except Exception as e:
            print(f"更新电池状态显示时出错: {str(e)}")
    
    def updatePositionDisplay(self, odometry_data):
        """更新位置信息显示"""
        try:
            if not odometry_data or not hasattr(self, 'position_label'):
                return
                
            # 获取位置数据
            pos_x = odometry_data["position"]["x"]
            pos_y = odometry_data["position"]["y"]
            pos_z = odometry_data["position"]["z"]
            
            # 更新位置标签，保留四位小数
            self.position_label.setText(f"Position: (X:{pos_x:.4f} Y:{pos_y:.4f} Z:{pos_z:.4f})")
            
            # 更新高度显示
            if hasattr(self, 'altitude_label'):
                self.altitude_label.setText(f"{pos_z:.4f} m")
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
            
            # 计算合成速度(cm/s)
            speed = velocity_data.get("speed", 0.0)
            
            # 保存速度数据以便在模拟模式下使用
            self.speed = int(speed)
            self.linear_speed = math.sqrt(linear_x**2 + linear_y**2)  # 地面速度
            
            # 更新仪表盘显示
            if hasattr(self, 'dashboard') and self.dashboard:
                self.dashboard.set_speed(self.speed)
                
                # 根据速度方向设置挡位
                if linear_x > 0.1:  # 前进
                    self.gear = 10  # D挡
                elif linear_x < -0.1:  # 后退
                    self.gear = 13  # R挡
                else:  # 静止
                    self.gear = 11  # N挡
                
                self.dashboard.set_gear(self.gear)
            
            # 更新地面速度标签
            if hasattr(self, 'ground_speed_label'):
                self.ground_speed_label.setText(f"{self.linear_speed:.4f} m/s")
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
            
            # 更新连接状态
            if hasattr(self, 'connection_label'):
                if connected:
                    self.connection_label.setText("已连接")
                    self.connection_label.setStyleSheet("color: #2ECC71;")
                else:
                    self.connection_label.setText("未连接")
                    self.connection_label.setStyleSheet("color: #E74C3C;")
            
            # 更新模式显示
            if hasattr(self, 'mode_label'):
                self.mode_label.setText(mode if mode else "UNKNOWN")
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
                    if self.current_image_mode == "rgb":
                        if not self.topic_subscriber or not self.topic_subscriber.is_topic_active("camera"):
                            self.image_label.setText("<div style='font-size: 16pt; color: #3498DB; text-align: center; margin-top: 200px;'>等待RGB图像话题连接...</div>")
                        else:
                            self.image_label.setText("<div style='font-size: 16pt; color: #3498DB; text-align: center; margin-top: 200px;'>等待RGB图像数据...</div>")
                    else:  # depth模式
                        if not self.topic_subscriber or not self.topic_subscriber.is_topic_active("depth"):
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
                ros_time = rospy.get_time()
                self.ros_time_label.setText(f"Time: {ros_time:.4f}")
            except:
                # 如果ROS节点未初始化，显示系统时间
                self.ros_time_label.setText(f"Time: {time.time():.4f}")
        
        # 如果没有话题订阅器或者话题还未连接，使用模拟数据更新
        if not self.topic_subscriber or (
            self.topic_subscriber and not (
                self.topic_subscriber.is_topic_active("battery") or 
                self.topic_subscriber.is_topic_active("odometry") or
                self.topic_subscriber.is_topic_active("velocity") or
                self.topic_subscriber.is_topic_active("status") or
                self.topic_subscriber.is_topic_active("camera")
            )
        ):
            # 模拟电池电量波动（仅用于测试）
            battery_percentage = self.battery_percentage + (random.uniform(-0.5, 0.3) if hasattr(random, 'uniform') else 0)
            # 确保百分比在合理范围内
            self.battery_percentage = max(0, min(100, battery_percentage))
            
            # 根据百分比模拟电压变化 - 与真实电池放电曲线类似
            # 12.6V (满电) -> 11.7V (空电)
            self.battery_voltage = 11.7 + (self.battery_percentage / 100.0) * 0.9
            self.voltage_label.setText(f"{self.battery_voltage:.2f} V")
            
            # 根据电量选择对应图标
            if self.battery_percentage <= 15:
                icon_path = ":/images/icons/battery_0.svg"
            elif self.battery_percentage <= 50:
                icon_path = ":/images/icons/battery_50.svg"
            elif self.battery_percentage <= 75:
                icon_path = ":/images/icons/battery_75.svg"
            else:
                icon_path = ":/images/icons/battery_100.svg"
                
            # 更新电池图标
            self.battery_icon_label.setPixmap(QPixmap(icon_path).scaled(24, 24, Qt.KeepAspectRatio, Qt.SmoothTransformation))
            
            # 模拟位置变化
            current_time = time.time()
            # 使用正弦和余弦函数生成圆形轨迹，加上一些小的随机波动
            pos_x = 10.0 * math.cos(current_time * 0.1) + random.uniform(-0.05, 0.05)
            pos_y = 10.0 * math.sin(current_time * 0.1) + random.uniform(-0.05, 0.05)
            pos_z = 2.0 + math.sin(current_time * 0.2) * 0.5 + random.uniform(-0.02, 0.02)
            
            # 更新位置显示
            self.position_label.setText(f"Position: (X:{pos_x:.4f} Y:{pos_y:.4f} Z:{pos_z:.4f})")
            
            # 更新高度显示
            if hasattr(self, 'altitude_label'):
                self.altitude_label.setText(f"{pos_z:.4f} m")
            
            # 模拟速度变化
            # 计算速度(与位置变化对应)
            speed_x = -10.0 * math.sin(current_time * 0.1) * 0.1  # dx/dt
            speed_y = 10.0 * math.cos(current_time * 0.1) * 0.1   # dy/dt
            speed_z = math.cos(current_time * 0.2) * 0.5 * 0.2    # dz/dt
            
            # 计算线速度和合成速度
            linear_speed = math.sqrt(speed_x**2 + speed_y**2)
            speed = math.sqrt(speed_x**2 + speed_y**2 + speed_z**2) * 100  # 转换为厘米/秒
            
            # 更新速度显示
            if hasattr(self, 'dashboard') and self.dashboard:
                self.dashboard.set_speed(int(speed))
                
                # 根据速度方向设置挡位
                if speed_x > 0.01:  # 前进
                    self.gear = 10  # D挡
                elif speed_x < -0.01:  # 后退
                    self.gear = 13  # R挡
                else:  # 静止
                    self.gear = 11  # N挡
                
                self.dashboard.set_gear(self.gear)
            
            # 更新地面速度标签
            if hasattr(self, 'ground_speed_label'):
                self.ground_speed_label.setText(f"{linear_speed:.4f} m/s")
            
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
            if (not self.topic_subscriber or not self.topic_subscriber.is_topic_active("camera")) and hasattr(self, 'image_label'):
                if self.current_image_mode == "rgb":  # 只在RGB模式下更新
                    self.image_label.setText("<div style='font-size: 16pt; color: #3498DB; text-align: center; margin-top: 200px;'>等待RGB图像话题连接...</div>")
                # 确保未使用模拟图像
                self.camera_image = None
                
            # 更新深度图像显示文本 - 使用自定义HTML样式显示
            if (not self.topic_subscriber or not self.topic_subscriber.is_topic_active("depth")) and hasattr(self, 'image_label'):
                if self.current_image_mode == "depth":  # 只在深度模式下更新
                    self.image_label.setText("<div style='font-size: 16pt; color: #3498DB; text-align: center; margin-top: 200px;'>等待深度图像话题连接...</div>")
                # 确保未使用模拟图像
                self.depth_image = None
                
            # 更新鸟瞰图显示文本 - 使用自定义HTML样式显示
            if (not self.topic_subscriber or not self.topic_subscriber.is_topic_active("bird_view")) and hasattr(self, 'bird_view_label'):
                self.bird_view_label.setText("<div style='font-size: 14pt; color: #3498DB; text-align: center; margin-top: 100px;'>等待鸟瞰图话题连接...</div>")
                # 确保未使用模拟图像
                self.bird_view_image = None
        
        # 每次渲染帧时增加计数器
        self.frame_count += 1

    def toggleRightSidebar(self):
        """显示或隐藏右侧栏"""
        if self.right_sidebar_expanded:
            # 隐藏右侧栏
            self.right_sidebar.setMaximumWidth(0)
            self.right_sidebar.setMinimumWidth(0)
            self.toggle_right_sidebar_btn.setIcon(QIcon(":/images/icons/dropleft.svg"))  # 切换图标为左箭头
            self.right_sidebar_expanded = False
            
            # 更新分割器尺寸，将右侧栏的空间分配给中间的RViz区域
            sizes = self.main_splitter.sizes()
            new_sizes = [sizes[0], sizes[1], sizes[2] + sizes[4], sizes[3], 0]
            self.main_splitter.setSizes(new_sizes)
        else:
            # 恢复右侧栏
            self.right_sidebar.setFixedWidth(650)  # 固定宽度650px
            self.toggle_right_sidebar_btn.setIcon(QIcon(":/images/icons/dropright.svg"))  # 切换图标为右箭头
            self.right_sidebar_expanded = True
            
            # 更新分割器尺寸，从中间区域分配空间给右侧栏
            sizes = self.main_splitter.sizes()
            if sizes[2] > 650:  # 确保中间区域有足够空间
                new_sizes = [sizes[0], sizes[1], sizes[2] - 650, sizes[3], 650]
            else:  # 如果中间区域空间不足，则按比例分配
                total_space = sizes[2]
                new_middle = max(int(total_space * 0.4), 100)  # 至少保留100px给中间区域
                new_sizes = [sizes[0], sizes[1], new_middle, sizes[3], total_space - new_middle]
            self.main_splitter.setSizes(new_sizes)

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
                        # 如果转换失败，尝试直接使用原始数据
                        q_image = QImage(self.bird_view_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
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
                        # 如果转换失败，创建一个空的图像
                        q_image = QImage(width, height, QImage.Format_RGB888)
                        q_image.fill(Qt.black)
                
                # 创建QPixmap并设置到标签
                try:
                    pixmap = QPixmap.fromImage(q_image)
                    
                    # 设置图像到标签，保持宽高比
                    self.bird_view_label.setPixmap(pixmap.scaled(
                        640, 
                        240,
                        Qt.KeepAspectRatio, 
                        Qt.SmoothTransformation
                    ))
                except Exception as e:
                    print(f"设置鸟瞰图像到UI失败: {str(e)}")
                    self.bird_view_label.setText("鸟瞰图显示错误")
            else:
                # 无图像时显示默认文本
                if hasattr(self, 'bird_view_label') and self.bird_view_label:
                    topic_active = self.topic_subscriber and self.topic_subscriber.is_topic_active("bird_view")
                    # 移除调试打印，减少控制台输出
                    if not topic_active:
                        self.bird_view_label.setText("<div style='font-size: 14pt; color: #3498DB; text-align: center; margin-top: 100px;'>等待鸟瞰图话题连接...</div>")
                    else:
                        self.bird_view_label.setText("<div style='font-size: 14pt; color: #3498DB; text-align: center; margin-top: 100px;'>等待鸟瞰图数据...</div>")
        except Exception as e:
            import traceback
            print(f"鸟瞰图显示更新错误: {str(e)}")
            print(traceback.format_exc())
            self.bird_view_label.setText(f"鸟瞰图显示错误: {str(e)}")
    
    # 添加人员位置管理功能
    def addPerson(self):
        """添加搜救人员位置"""
        # 创建对话框
        dialog = QDialog(self)
        dialog.setWindowTitle("添加待搜救人员")
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
                
                print(f"已添加新的搜救人员: ID={next_id}, X={x_value}, Y={y_value}, 状态={status}")
            except Exception as e:
                print(f"添加人员时出错: {str(e)}")
                QMessageBox.critical(self, "错误", f"添加人员时出错: {str(e)}")
    
    def removePerson(self):
        """删除选中的搜救人员"""
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
            self.position_table.setItem(row_position, 1, QTableWidgetItem(f"{x:.2f}"))
            self.position_table.setItem(row_position, 2, QTableWidgetItem(f"{y:.2f}"))
            
            # 设置状态为"待确认"
            status_item = QTableWidgetItem("待确认")
            status_item.setForeground(QBrush(QColor("#F39C12")))  # 橙色
            self.position_table.setItem(row_position, 3, status_item)
            
            print(f"已添加标记点到表格: ID={ball_id}, X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
            
            # 自动滚动到新添加的行
            self.position_table.scrollToItem(self.position_table.item(row_position, 0))
        except Exception as e:
            print(f"添加标记点到表格时出错: {str(e)}")

## Start the Application
## ^^^^^^^^^^^^^^^^^^^^^
##
## That is just about it.  All that's left is the standard Qt
## top-level application code: create a QApplication, instantiate our
## class, and start Qt's main event loop (app.exec_()).
if __name__ == '__main__':
    app = QApplication(sys.argv)
    
    # 确保应用程序支持中文
    QTextCodec.setCodecForLocale(QTextCodec.codecForName("UTF-8"))
    
    # 初始化ROS节点
    try:
        rospy.init_node('myviz', anonymous=True)
        print("成功初始化ROS节点: myviz")
    except Exception as e:
        print(f"警告: ROS节点初始化失败: {str(e)}")
    
    # 创建主窗口
    myviz = MyViz()
    
            # 获取可用屏幕区域（考虑任务栏/面板）
    desktop = QDesktopWidget()
    available_geometry = desktop.availableGeometry(desktop.primaryScreen())
    width = available_geometry.width()
    height = available_geometry.height()
    print(f"可用屏幕区域: {width}x{height}, 位置: ({available_geometry.x()}, {available_geometry.y()})")
    
    # 直接以最大化模式启动窗口
    myviz.showMaximized()
    
    # 不再需要单独调用show()，showMaximized()已经包含了show()功能
    # myviz.show()
    
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
            
        sys.exit(0)
