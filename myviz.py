#!/usr/bin/env python
# -*- coding: utf-8 -*-

## BEGIN_TUTORIAL
##
## Imports
## ^^^^^^^
##
## First we start with the standard ros Python import line:
import roslib; roslib.load_manifest('rviz_python_tutorial')

## Then load sys to get sys.argv.
import sys

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

## Finally import the RViz bindings themselves.
from rviz import bindings as rviz

## The MyViz class is the main container widget.
class MyViz( QWidget ):

    ## MyViz Constructor
    ## ^^^^^^^^^^^^^^^^^
    ##
    ## Its constructor creates and configures all the component widgets:
    ## frame, thickness_slider, top_button, and side_button, and adds them
    ## to layouts.
    def __init__(self):
        QWidget.__init__(self)
        
        # 设置中文字体支持
        font = QFont("WenQuanYi Micro Hei", 10)
        QApplication.setFont(font)
        
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
        
        # 设置窗口标志，使标题栏更宽
        self.setWindowFlags(Qt.Window | Qt.CustomizeWindowHint | Qt.WindowTitleHint | 
                           Qt.WindowSystemMenuHint | Qt.WindowMinMaxButtonsHint | Qt.WindowCloseButtonHint)
        
        ## rviz.VisualizationFrame是RViz应用程序的主容器窗口小部件
        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath("")
        self.frame.initialize()

        ## 读取配置文件
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile(config, "my_config.rviz")
        self.frame.load(config)

        ## 设置窗口标题
        self.setWindowTitle("无人机自主搜救系统")

        ## 禁用菜单栏、状态栏和"隐藏停靠"按钮
        self.frame.setMenuBar(None)
        self.frame.setStatusBar(None)
        self.frame.setHideButtonVisibility(True)

        ## 获取VisualizationManager实例
        self.manager = self.frame.getManager()
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt(0)
        
        ## 创建主布局
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.setSpacing(5)
        
        ## 创建标题和工具栏区域
        header_widget = QWidget()
        header_widget.setStyleSheet("background-color: #1A202C; border-radius: 5px;")
        header_widget.setMaximumHeight(100)  # 限制标题栏最大高度
        header_layout = QVBoxLayout(header_widget)
        header_layout.setContentsMargins(5, 5, 5, 5)
        header_layout.setSpacing(2)
        
        # 创建标题标签
        title_label = QLabel("无人机自主搜救系统")
        title_label.setStyleSheet("font-size: 16pt; color: #3498DB; padding: 5px;")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setMaximumHeight(40)  # 限制标题标签高度
        header_layout.addWidget(title_label)
        
        # 创建工具栏
        toolbar_widget = QWidget()
        toolbar_layout = QHBoxLayout(toolbar_widget)
        toolbar_layout.setContentsMargins(5, 0, 5, 0)
        toolbar_layout.setSpacing(10)
        
        # 视图控制按钮
        view_layout = QHBoxLayout()
        view_layout.setSpacing(5)
        view_label = QLabel("视图:")
        view_label.setStyleSheet("font-size: 10pt; padding: 0px;")
        view_label.setMaximumWidth(40)
        view_layout.addWidget(view_label)
        
        top_button = QPushButton("俯视图")
        top_button.setMinimumWidth(80)
        top_button.setMaximumHeight(30)
        top_button.clicked.connect(self.onTopButtonClick)
        view_layout.addWidget(top_button)
        
        side_button = QPushButton("侧视图")
        side_button.setMinimumWidth(80)
        side_button.setMaximumHeight(30)
        side_button.clicked.connect(self.onSideButtonClick)
        view_layout.addWidget(side_button)
        
        free_button = QPushButton("自由视图")
        free_button.setMinimumWidth(80)
        free_button.setMaximumHeight(30)
        free_button.clicked.connect(self.onFreeViewClick)
        view_layout.addWidget(free_button)
        
        toolbar_layout.addLayout(view_layout)
        toolbar_layout.addStretch(1)
        
        # 功能按钮
        function_layout = QHBoxLayout()
        function_layout.setSpacing(5)
        
        start_button = QPushButton("开始搜救")
        start_button.setStyleSheet("background-color: #27AE60;")
        start_button.setMaximumHeight(30)
        function_layout.addWidget(start_button)
        
        stop_button = QPushButton("停止")
        stop_button.setStyleSheet("background-color: #E74C3C;")
        stop_button.setMaximumHeight(30)
        function_layout.addWidget(stop_button)
        
        reset_button = QPushButton("重置")
        reset_button.setMaximumHeight(30)
        function_layout.addWidget(reset_button)
        
        # 设置按钮，用于显示/隐藏Display面板
        self.settings_button = QPushButton("设置")
        self.settings_button.setMaximumHeight(30)
        self.settings_button.clicked.connect(self.toggleDisplayPanel)
        function_layout.addWidget(self.settings_button)
        
        toolbar_layout.addLayout(function_layout)
        
        header_layout.addWidget(toolbar_widget)
        main_layout.addWidget(header_widget)
        
        # 创建中间的分割器，用于显示RViz和Display面板
        self.splitter = QSplitter(Qt.Horizontal)
        
        # 添加RViz框架
        self.splitter.addWidget(self.frame)
        
        # 创建Display面板
        self.display_panel = QWidget()
        self.display_panel.setMinimumWidth(300)
        self.display_panel.setMaximumWidth(400)
        display_layout = QVBoxLayout(self.display_panel)
        
        # 获取RViz的Display面板
        # 由于getDisplaysPanel方法不存在，我们需要创建自己的显示面板
        display_group = QGroupBox("显示设置")
        display_group.setStyleSheet("color: #3498DB;")
        display_group_layout = QVBoxLayout(display_group)
        
        # 创建一个树形控件来模拟Display面板
        display_tree = QTreeWidget()
        display_tree.setHeaderLabel("显示选项")
        display_tree.setStyleSheet("""
            background-color: #1E2330; 
            color: white;
        """)
        display_tree.header().setDefaultAlignment(Qt.AlignCenter)  # 设置标题居中
        
        # 添加一些基本选项
        root_item = QTreeWidgetItem(display_tree, ["全局选项"])
        root_item.setTextAlignment(0, Qt.AlignCenter)  # 设置项目文本居中
        
        grid_item = QTreeWidgetItem(root_item, ["网格"])
        grid_item.setTextAlignment(0, Qt.AlignCenter)
        
        planning_item = QTreeWidgetItem(display_tree, ["规划"])
        planning_item.setTextAlignment(0, Qt.AlignCenter)
        
        images_item = QTreeWidgetItem(display_tree, ["图像"])
        images_item.setTextAlignment(0, Qt.AlignCenter)
        
        # 添加图像子项
        uav_image_item = QTreeWidgetItem(images_item, ["无人机图像"])
        uav_image_item.setTextAlignment(0, Qt.AlignCenter)
        
        depth_image_item = QTreeWidgetItem(images_item, ["深度图像"])
        depth_image_item.setTextAlignment(0, Qt.AlignCenter)
        
        bird_image_item = QTreeWidgetItem(images_item, ["鸟瞰图"])
        bird_image_item.setTextAlignment(0, Qt.AlignCenter)
        
        udepth_image_item = QTreeWidgetItem(images_item, ["U-深度图像"])
        udepth_image_item.setTextAlignment(0, Qt.AlignCenter)
        
        pointcloud_item = QTreeWidgetItem(display_tree, ["点云"])
        pointcloud_item.setTextAlignment(0, Qt.AlignCenter)
        
        odom_item = QTreeWidgetItem(display_tree, ["里程计"])
        odom_item.setTextAlignment(0, Qt.AlignCenter)
        
        display_tree.expandAll()
        display_group_layout.addWidget(display_tree)
        
        # 添加一个刷新按钮
        refresh_button = QPushButton("刷新")
        display_group_layout.addWidget(refresh_button)
        
        display_layout.addWidget(display_group)
        
        # 默认隐藏Display面板
        self.display_panel.setVisible(False)
        self.splitter.addWidget(self.display_panel)
        
        main_layout.addWidget(self.splitter)
        
        # 创建底部状态栏，包含Time信息
        status_bar = QStatusBar()
        status_bar.setStyleSheet("background-color: #1A202C; padding: 2px;")
        status_bar.setMaximumHeight(25)  # 限制底部状态栏高度
        
        # 获取RViz的Time面板，如果方法不存在则创建一个自定义的时间显示
        try:
            self.time_panel = self.frame.getTimePanel()
            if self.time_panel:
                self.time_panel.setMaximumHeight(20)  # 限制时间面板高度
                status_bar.addPermanentWidget(self.time_panel)
        except AttributeError:
            # 创建自定义时间显示
            time_label = QLabel("时间: 0.00")
            time_label.setStyleSheet("color: #3498DB;")
            status_bar.addPermanentWidget(time_label)
        
        main_layout.addWidget(status_bar)
        
        # 设置布局的拉伸因子，让中间的RViz显示区域占据大部分空间
        main_layout.setStretch(1, 10)  # 中间部分(splitter)占据更多空间
        main_layout.setStretch(0, 1)   # 标题栏占据较少空间
        main_layout.setStretch(2, 1)   # 底部栏占据较少空间
        
        self.setLayout(main_layout)
    
    def toggleDisplayPanel(self):
        """显示或隐藏Display面板"""
        self.display_panel.setVisible(not self.display_panel.isVisible())
        if self.display_panel.isVisible():
            self.settings_button.setText("隐藏设置")
        else:
            self.settings_button.setText("设置")

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
    
    myviz = MyViz()
    # 设置窗口全屏显示
    desktop = QApplication.desktop()
    screen_rect = desktop.screenGeometry()
    myviz.resize(screen_rect.width(), screen_rect.height())
    myviz.show()

    app.exec_()
