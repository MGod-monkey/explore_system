#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
目标点设置对话框模块
用于设置无人机导航目标点，支持多目标点添加和保存/加载
"""

import os
import json
import rospy
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

from geometry_msgs.msg import PoseStamped


class WaypointDialog(QDialog):
    """目标点设置对话框"""
    
    # 信号：当开始导航时发出
    navigationStarted = pyqtSignal(list)  # 发送目标点列表
    
    def __init__(self, parent=None, topic_subscriber=None):
        super(WaypointDialog, self).__init__(parent)
        self.topic_subscriber = topic_subscriber
        self.waypoints = []  # 目标点列表
        self.current_waypoint_index = 0  # 当前目标点索引
        self.is_navigating = False  # 是否正在导航
        
        # 设置窗口标志 - 非模态窗口，可以拖动和缩放
        self.setWindowFlags(Qt.Window | Qt.WindowMinMaxButtonsHint | Qt.WindowCloseButtonHint)
        self.setAttribute(Qt.WA_DeleteOnClose, False)  # 关闭时不删除
        
        # 目标点发布者
        self.goal_publisher = None
        try:
            self.goal_publisher = rospy.Publisher(
                '/move_base_simple/goal',
                PoseStamped,
                queue_size=10
            )
        except Exception as e:
            print(f"初始化目标点发布者失败: {e}")
        
        # FSM状态检查定时器
        self.fsm_check_timer = QTimer(self)
        self.fsm_check_timer.timeout.connect(self.checkFSMState)
        
        self.setupUI()
        self.loadDefaultWaypoints()
    
    def setupUI(self):
        """设置UI界面"""
        self.setWindowTitle("目标点设置")
        self.setMinimumSize(500, 400)
        self.setStyleSheet("""
            QDialog {
                background-color: #1E2330;
                color: #FFFFFF;
            }
            QLabel {
                color: #FFFFFF;
                font-size: 11pt;
            }
            QLineEdit {
                background-color: #2C3E50;
                color: #FFFFFF;
                border: 1px solid #3498DB;
                border-radius: 4px;
                padding: 5px;
                font-size: 11pt;
            }
            QPushButton {
                background-color: #3498DB;
                color: white;
                border-radius: 4px;
                padding: 8px 16px;
                font-size: 11pt;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #2980B9;
            }
            QPushButton:pressed {
                background-color: #1A5276;
            }
            QPushButton#addBtn {
                background-color: #27AE60;
            }
            QPushButton#addBtn:hover {
                background-color: #229954;
            }
            QPushButton#deleteBtn {
                background-color: #E74C3C;
            }
            QPushButton#deleteBtn:hover {
                background-color: #C0392B;
            }
            QPushButton#startBtn {
                background-color: #27AE60;
                min-height: 40px;
            }
            QPushButton#startBtn:hover {
                background-color: #229954;
            }
            QTableWidget {
                background-color: #2C3E50;
                color: #FFFFFF;
                border: 1px solid #3498DB;
                border-radius: 4px;
                gridline-color: #3498DB;
            }
            QTableWidget::item {
                padding: 5px;
            }
            QTableWidget::item:selected {
                background-color: #3498DB;
            }
            QHeaderView::section {
                background-color: #1A202C;
                color: #FFFFFF;
                padding: 8px;
                border: 1px solid #3498DB;
                font-weight: bold;
            }
            QGroupBox {
                color: #3498DB;
                font-weight: bold;
                border: 1px solid #3498DB;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top center;
                padding: 0 5px;
            }
        """)
        
        main_layout = QVBoxLayout(self)
        main_layout.setSpacing(10)
        
        # 状态显示区域
        status_group = QGroupBox("导航状态")
        status_layout = QHBoxLayout(status_group)
        
        self.status_label = QLabel("等待开始导航")
        self.status_label.setStyleSheet("color: #BDC3C7; font-weight: bold;")
        status_layout.addWidget(self.status_label)
        
        self.fsm_label = QLabel("FSM: 未知")
        self.fsm_label.setStyleSheet("color: #F39C12;")
        status_layout.addWidget(self.fsm_label)
        
        main_layout.addWidget(status_group)
        
        # 坐标输入区域
        input_group = QGroupBox("添加目标点")
        input_layout = QHBoxLayout(input_group)
        
        # X坐标
        input_layout.addWidget(QLabel("X:"))
        self.x_input = QLineEdit("0.0")
        self.x_input.setMaximumWidth(80)
        self.x_input.setValidator(QDoubleValidator(-1000, 1000, 2))
        input_layout.addWidget(self.x_input)
        
        # Y坐标
        input_layout.addWidget(QLabel("Y:"))
        self.y_input = QLineEdit("0.0")
        self.y_input.setMaximumWidth(80)
        self.y_input.setValidator(QDoubleValidator(-1000, 1000, 2))
        input_layout.addWidget(self.y_input)
        
        # Z坐标
        input_layout.addWidget(QLabel("Z:"))
        self.z_input = QLineEdit("1.0")
        self.z_input.setMaximumWidth(80)
        self.z_input.setValidator(QDoubleValidator(-100, 100, 2))
        input_layout.addWidget(self.z_input)
        
        # 添加按钮
        self.add_btn = QPushButton("添加")
        self.add_btn.setObjectName("addBtn")
        self.add_btn.clicked.connect(self.addWaypoint)
        input_layout.addWidget(self.add_btn)
        
        main_layout.addWidget(input_group)
        
        # 目标点列表
        list_group = QGroupBox("目标点列表")
        list_layout = QVBoxLayout(list_group)
        
        self.waypoint_table = QTableWidget()
        self.waypoint_table.setColumnCount(4)
        self.waypoint_table.setHorizontalHeaderLabels(["序号", "X", "Y", "Z"])
        self.waypoint_table.horizontalHeader().setStretchLastSection(True)
        self.waypoint_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.waypoint_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.waypoint_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        list_layout.addWidget(self.waypoint_table)
        
        # 列表操作按钮
        list_btn_layout = QHBoxLayout()
        
        self.delete_btn = QPushButton("删除选中")
        self.delete_btn.setObjectName("deleteBtn")
        self.delete_btn.clicked.connect(self.deleteSelectedWaypoint)
        list_btn_layout.addWidget(self.delete_btn)
        
        self.clear_btn = QPushButton("清空列表")
        self.clear_btn.setObjectName("deleteBtn")
        self.clear_btn.clicked.connect(self.clearWaypoints)
        list_btn_layout.addWidget(self.clear_btn)
        
        self.move_up_btn = QPushButton("上移")
        self.move_up_btn.clicked.connect(self.moveWaypointUp)
        list_btn_layout.addWidget(self.move_up_btn)
        
        self.move_down_btn = QPushButton("下移")
        self.move_down_btn.clicked.connect(self.moveWaypointDown)
        list_btn_layout.addWidget(self.move_down_btn)
        
        list_layout.addLayout(list_btn_layout)
        main_layout.addWidget(list_group)
        
        # 文件操作按钮
        file_layout = QHBoxLayout()
        
        self.save_btn = QPushButton("保存目标点")
        self.save_btn.clicked.connect(self.saveWaypoints)
        file_layout.addWidget(self.save_btn)
        
        self.load_btn = QPushButton("加载目标点")
        self.load_btn.clicked.connect(self.loadWaypoints)
        file_layout.addWidget(self.load_btn)
        
        main_layout.addLayout(file_layout)
        
        # 导航控制按钮
        nav_layout = QHBoxLayout()
        
        self.start_btn = QPushButton("开始导航")
        self.start_btn.setObjectName("startBtn")
        self.start_btn.clicked.connect(self.startNavigation)
        nav_layout.addWidget(self.start_btn)
        
        self.stop_btn = QPushButton("停止导航")
        self.stop_btn.setObjectName("deleteBtn")
        self.stop_btn.clicked.connect(self.stopNavigation)
        self.stop_btn.setEnabled(False)
        nav_layout.addWidget(self.stop_btn)
        
        main_layout.addLayout(nav_layout)
        
        # 关闭按钮
        close_btn = QPushButton("关闭")
        close_btn.clicked.connect(self.close)
        main_layout.addWidget(close_btn)
    
    def addWaypoint(self):
        """添加目标点"""
        try:
            x = float(self.x_input.text())
            y = float(self.y_input.text())
            z = float(self.z_input.text())
            
            self.waypoints.append({'x': x, 'y': y, 'z': z})
            self.updateWaypointTable()
            
            # 清空输入框
            self.x_input.clear()
            self.y_input.clear()
            self.z_input.setText("1.0")
            
        except ValueError:
            QMessageBox.warning(self, "输入错误", "请输入有效的坐标值")
    
    def updateWaypointTable(self):
        """更新目标点表格"""
        self.waypoint_table.setRowCount(len(self.waypoints))
        for i, wp in enumerate(self.waypoints):
            # 序号
            item_idx = QTableWidgetItem(str(i + 1))
            item_idx.setTextAlignment(Qt.AlignCenter)
            self.waypoint_table.setItem(i, 0, item_idx)
            
            # X坐标
            item_x = QTableWidgetItem(f"{wp['x']:.2f}")
            item_x.setTextAlignment(Qt.AlignCenter)
            self.waypoint_table.setItem(i, 1, item_x)
            
            # Y坐标
            item_y = QTableWidgetItem(f"{wp['y']:.2f}")
            item_y.setTextAlignment(Qt.AlignCenter)
            self.waypoint_table.setItem(i, 2, item_y)
            
            # Z坐标
            item_z = QTableWidgetItem(f"{wp['z']:.2f}")
            item_z.setTextAlignment(Qt.AlignCenter)
            self.waypoint_table.setItem(i, 3, item_z)
            
            # 高亮当前正在执行的目标点
            if self.is_navigating and i == self.current_waypoint_index:
                for col in range(4):
                    self.waypoint_table.item(i, col).setBackground(QColor("#27AE60"))
    
    def deleteSelectedWaypoint(self):
        """删除选中的目标点"""
        selected_rows = set()
        for item in self.waypoint_table.selectedItems():
            selected_rows.add(item.row())
        
        for row in sorted(selected_rows, reverse=True):
            if row < len(self.waypoints):
                del self.waypoints[row]
        
        self.updateWaypointTable()
    
    def clearWaypoints(self):
        """清空目标点列表"""
        if self.waypoints:
            reply = QMessageBox.question(self, "确认清空", 
                                         "确定要清空所有目标点吗？",
                                         QMessageBox.Yes | QMessageBox.No,
                                         QMessageBox.No)
            if reply == QMessageBox.Yes:
                self.waypoints.clear()
                self.updateWaypointTable()
    
    def moveWaypointUp(self):
        """上移选中的目标点"""
        current_row = self.waypoint_table.currentRow()
        if current_row > 0:
            self.waypoints[current_row], self.waypoints[current_row - 1] = \
                self.waypoints[current_row - 1], self.waypoints[current_row]
            self.updateWaypointTable()
            self.waypoint_table.selectRow(current_row - 1)
    
    def moveWaypointDown(self):
        """下移选中的目标点"""
        current_row = self.waypoint_table.currentRow()
        if current_row < len(self.waypoints) - 1:
            self.waypoints[current_row], self.waypoints[current_row + 1] = \
                self.waypoints[current_row + 1], self.waypoints[current_row]
            self.updateWaypointTable()
            self.waypoint_table.selectRow(current_row + 1)
    
    def saveWaypoints(self):
        """保存目标点到文件"""
        if not self.waypoints:
            QMessageBox.warning(self, "保存失败", "没有可保存的目标点")
            return
        
        filename, _ = QFileDialog.getSaveFileName(
            self, "保存目标点", "", "JSON文件 (*.json);;所有文件 (*)"
        )
        
        if filename:
            try:
                with open(filename, 'w') as f:
                    json.dump(self.waypoints, f, indent=2)
                QMessageBox.information(self, "保存成功", f"目标点已保存到:\n{filename}")
            except Exception as e:
                QMessageBox.critical(self, "保存失败", f"保存文件时出错:\n{str(e)}")
    
    def loadWaypoints(self):
        """从文件加载目标点"""
        filename, _ = QFileDialog.getOpenFileName(
            self, "加载目标点", "", "JSON文件 (*.json);;所有文件 (*)"
        )
        
        if filename:
            try:
                with open(filename, 'r') as f:
                    loaded_waypoints = json.load(f)
                
                # 验证数据格式
                for wp in loaded_waypoints:
                    if not all(key in wp for key in ['x', 'y', 'z']):
                        raise ValueError("目标点格式无效")
                
                self.waypoints = loaded_waypoints
                self.updateWaypointTable()
                QMessageBox.information(self, "加载成功", f"已加载 {len(self.waypoints)} 个目标点")
            except Exception as e:
                QMessageBox.critical(self, "加载失败", f"加载文件时出错:\n{str(e)}")
    
    def loadDefaultWaypoints(self):
        """加载默认目标点文件"""
        default_file = os.path.join(os.path.dirname(__file__), "waypoints.json")
        if os.path.exists(default_file):
            try:
                with open(default_file, 'r') as f:
                    self.waypoints = json.load(f)
                self.updateWaypointTable()
            except Exception:
                pass
    
    def startNavigation(self):
        """开始导航"""
        if not self.waypoints:
            QMessageBox.warning(self, "无法导航", "请先添加目标点")
            return
        
        reply = QMessageBox.question(self, "确认导航", 
                                     f"确定要开始导航到 {len(self.waypoints)} 个目标点吗？",
                                     QMessageBox.Yes | QMessageBox.No,
                                     QMessageBox.No)
        if reply != QMessageBox.Yes:
            return
        
        self.is_navigating = True
        self.current_waypoint_index = 0
        
        # 更新UI状态
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.add_btn.setEnabled(False)
        self.delete_btn.setEnabled(False)
        self.clear_btn.setEnabled(False)
        
        self.status_label.setText(f"正在导航到目标点 1/{len(self.waypoints)}")
        self.status_label.setStyleSheet("color: #27AE60; font-weight: bold;")
        
        # 发送第一个目标点
        self.publishCurrentWaypoint()
        
        # 启动FSM状态检查定时器
        self.fsm_check_timer.start(500)  # 每500ms检查一次
        
        self.updateWaypointTable()
    
    def stopNavigation(self):
        """停止导航"""
        self.is_navigating = False
        self.fsm_check_timer.stop()
        
        # 更新UI状态
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.add_btn.setEnabled(True)
        self.delete_btn.setEnabled(True)
        self.clear_btn.setEnabled(True)
        
        self.status_label.setText("导航已停止")
        self.status_label.setStyleSheet("color: #E74C3C; font-weight: bold;")
        
        self.updateWaypointTable()
    
    def publishCurrentWaypoint(self):
        """发布当前目标点"""
        if not self.is_navigating or self.current_waypoint_index >= len(self.waypoints):
            return
        
        if self.goal_publisher is None:
            print("目标点发布者未初始化")
            return
        
        wp = self.waypoints[self.current_waypoint_index]
        
        # 创建目标点消息
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "world"
        goal_msg.header.stamp = rospy.Time.now()
        
        goal_msg.pose.position.x = wp['x']
        goal_msg.pose.position.y = wp['y']
        goal_msg.pose.position.z = wp['z']
        
        # 设置朝向（默认向前）
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        
        # 发布目标点
        try:
            self.goal_publisher.publish(goal_msg)
            print(f"已发布目标点 {self.current_waypoint_index + 1}: ({wp['x']}, {wp['y']}, {wp['z']})")
        except Exception as e:
            print(f"发布目标点失败: {e}")
    
    def checkFSMState(self):
        """检查FSM状态，判断是否需要发送下一个目标点"""
        if not self.is_navigating:
            return
        
        # 获取FSM状态
        fsm_state = 0
        fsm_state_name = "UNKNOWN"
        
        if self.topic_subscriber:
            fsm_data = self.topic_subscriber.get_data("fsm_state")
            if fsm_data:
                fsm_state = fsm_data.get("state", 0)
                fsm_state_name = fsm_data.get("state_name", "UNKNOWN")
        
        # 更新FSM状态显示
        self.fsm_label.setText(f"FSM: {fsm_state_name}")
        
        # 根据FSM状态设置颜色
        if fsm_state == 1:  # WAIT_TARGET
            self.fsm_label.setStyleSheet("color: #27AE60;")  # 绿色 - 等待目标
        elif fsm_state == 4:  # EXEC_TRAJ
            self.fsm_label.setStyleSheet("color: #3498DB;")  # 蓝色 - 执行中
        elif fsm_state == 5:  # EMERGENCY_STOP
            self.fsm_label.setStyleSheet("color: #E74C3C;")  # 红色 - 紧急停止
        else:
            self.fsm_label.setStyleSheet("color: #F39C12;")  # 橙色 - 其他状态
        
        # 检查是否需要发送下一个目标点
        # 当FSM状态为 WAIT_TARGET (1) 时，说明无人机已到达当前目标点，可以发送下一个
        if fsm_state == 1:  # WAIT_TARGET
            # 移动到下一个目标点
            self.current_waypoint_index += 1
            
            if self.current_waypoint_index < len(self.waypoints):
                # 还有更多目标点
                self.status_label.setText(f"正在导航到目标点 {self.current_waypoint_index + 1}/{len(self.waypoints)}")
                self.publishCurrentWaypoint()
                self.updateWaypointTable()
            else:
                # 所有目标点已完成
                self.navigationComplete()
    
    def navigationComplete(self):
        """导航完成"""
        self.is_navigating = False
        self.fsm_check_timer.stop()
        
        # 更新UI状态
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.add_btn.setEnabled(True)
        self.delete_btn.setEnabled(True)
        self.clear_btn.setEnabled(True)
        
        self.status_label.setText("所有目标点已完成!")
        self.status_label.setStyleSheet("color: #27AE60; font-weight: bold;")
        
        self.updateWaypointTable()
        
        QMessageBox.information(self, "导航完成", "已成功到达所有目标点!")
    
    def closeEvent(self, event):
        """关闭事件"""
        if self.is_navigating:
            reply = QMessageBox.question(self, "确认关闭",
                                         "正在导航中，确定要关闭吗？",
                                         QMessageBox.Yes | QMessageBox.No,
                                         QMessageBox.No)
            if reply == QMessageBox.No:
                event.ignore()
                return
            
            self.stopNavigation()
        
        event.accept()
