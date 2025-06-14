#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading
import time
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *
import rostopic
import genpy
import yaml
import json
import numpy as np

class TopicLoggerDialog(QDialog):
    """ROS话题数据记录器对话框，可以显示选定话题的消息内容"""
    
    # 用于存储所有窗口实例的列表，便于统一管理
    all_windows = []
    
    def __init__(self, parent=None):
        super(TopicLoggerDialog, self).__init__(parent)
        self.setWindowTitle("ROS话题数据监控")
        self.setWindowIcon(QIcon(":/images/icons/log.svg"))
        self.setMinimumSize(800, 600)
        
        # 设置窗口背景颜色
        self.setStyleSheet("""
            QDialog {
                background-color: #1E2330;
                color: #FFFFFF;
            }
            QLabel {
                color: #FFFFFF;
            }
        """)
        
        # 创建主布局
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)
        
        # 创建顶部工具条，用于放置新窗口按钮
        toolbar = QHBoxLayout()
        toolbar.setContentsMargins(0, 0, 0, 5)
        
        # 创建"新窗口"按钮
        new_window_btn = QPushButton("新窗口")
        new_window_btn.setIcon(QIcon(":/images/icons/log.svg"))
        new_window_btn.setToolTip("创建一个新的独立日志窗口")
        new_window_btn.setStyleSheet("""
            QPushButton {
                background-color: #2980B9;
                color: white;
                border-radius: 4px;
                padding: 5px;
                font-weight: bold;
                min-width: 90px;
                min-height: 25px;
            }
            QPushButton:hover {
                background-color: #3498DB;
            }
            QPushButton:pressed {
                background-color: #2472A4;
            }
        """)
        new_window_btn.clicked.connect(self.create_new_window)
        toolbar.addWidget(new_window_btn)
        
        # 添加弹性空间，把工具条按钮推到左边
        toolbar.addStretch(1)
        
        # 添加工具条到主布局
        main_layout.addLayout(toolbar)
        
        # 创建内部的日志组件
        self.logger_widget = TopicLogger(self)
        main_layout.addWidget(self.logger_widget)
        
        # 设置对话框属性
        self.setWindowFlags(self.windowFlags() | Qt.Window)
        
        # 将自己添加到窗口列表
        TopicLoggerDialog.all_windows.append(self)
        
        # 标记窗口是否正在关闭，避免多次调用清理代码
        self.is_closing = False
        
    def create_new_window(self):
        """创建一个新的独立日志窗口"""
        try:
            # 创建新窗口
            new_dialog = TopicLoggerDialog()
            new_dialog.show()
            # 不需要手动添加到窗口列表，构造函数会自动添加
        except Exception as e:
            rospy.logerr(f"创建新窗口时出错: {str(e)}")
        
    def closeEvent(self, event):
        """处理窗口关闭事件"""
        try:
            # 如果已经在关闭过程中，不重复执行
            if self.is_closing:
                event.accept()
                return
                
            self.is_closing = True
            
            # 安全地停止记录
            if hasattr(self.logger_widget, 'stop_logging'):
                try:
                    self.logger_widget.stop_logging()
                except Exception as e:
                    rospy.logerr(f"停止日志记录时出错: {str(e)}")
                    
            # 安全关闭话题订阅
            if hasattr(self.logger_widget, 'subscriber') and self.logger_widget.subscriber:
                try:
                    self.logger_widget.subscriber.unregister()
                    self.logger_widget.subscriber = None
                except Exception as e:
                    rospy.logerr(f"取消话题订阅时出错: {str(e)}")
                    
            # 从全局窗口列表中移除自己
            if self in TopicLoggerDialog.all_windows:
                TopicLoggerDialog.all_windows.remove(self)
                
            event.accept()
        except Exception as e:
            rospy.logerr(f"关闭窗口时出错: {str(e)}")
            # 确保窗口关闭，即使出现异常
            event.accept()
        
    def shutdown(self):
        """关闭组件，清理资源"""
        try:
            # 如果已经在关闭过程中，不重复执行
            if self.is_closing:
                return
                
            self.is_closing = True
            
            if hasattr(self.logger_widget, 'shutdown'):
                self.logger_widget.shutdown()
        except Exception as e:
            rospy.logerr(f"关闭日志组件时出错: {str(e)}")
    
    @classmethod
    def close_all_windows(cls):
        """关闭所有日志窗口，用于程序退出时清理"""
        for window in list(cls.all_windows):
            try:
                window.shutdown()
                window.close()
            except Exception as e:
                rospy.logerr(f"关闭日志窗口时出错: {str(e)}")
        cls.all_windows.clear()


class TopicLogger(QWidget):
    """ROS话题数据记录器组件，可以显示选定话题的消息内容"""
    
    # 信号定义，用于线程安全地更新UI
    topic_message_signal = pyqtSignal(str)
    topics_updated_signal = pyqtSignal(list)
    
    def __init__(self, parent=None):
        """初始化话题数据记录器组件"""
        super(TopicLogger, self).__init__(parent)
        
        # 当前订阅的话题和订阅者
        self.current_topic = None
        self.subscriber = None
        
        # 话题列表
        self.topics_list = []
        
        # 是否正在记录
        self.is_logging = False
        
        # 标记是否正在关闭
        self.is_shutting_down = False
        
        # 创建UI
        self.setupUI()
        
        # 启动话题监控线程
        self.running = True
        self.topic_monitor_thread = threading.Thread(target=self.monitor_topics)
        self.topic_monitor_thread.daemon = True
        self.topic_monitor_thread.start()
        
        # 连接信号与槽
        self.topic_message_signal.connect(self.update_log_text)
        self.topics_updated_signal.connect(self.update_topics_list)
        
    def setupUI(self):
        """设置UI布局和组件"""
        # 使用垂直布局
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(5, 5, 5, 5)  # 设置较小的边距
        
        # 设置组件背景色
        self.setStyleSheet("""
            QWidget {
                background-color: #1E2330;
                color: #FFFFFF;
            }
        """)
        
        # 创建标题
        title_label = QLabel("ROS话题数据监控")
        title_label.setStyleSheet("font-size: 14pt; color: #3498DB; font-weight: bold;")
        title_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title_label)
        
        # 创建顶部控制区域
        top_widget = QWidget()
        top_widget.setStyleSheet("background-color: #1E2330;")
        top_layout = QHBoxLayout(top_widget)
        top_layout.setContentsMargins(0, 0, 0, 0)
        
        # 创建话题选择下拉框
        self.topic_combo = QComboBox()
        self.topic_combo.setStyleSheet("""
            QComboBox {
                background-color: #2C3E50;
                color: white;
                padding: 5px;
                border: 1px solid #3498DB;
                border-radius: 4px;
                min-height: 25px;
            }
            QComboBox::drop-down {
                subcontrol-origin: padding;
                subcontrol-position: top right;
                width: 20px;
                border-left: 1px solid #3498DB;
                border-top-right-radius: 3px;
                border-bottom-right-radius: 3px;
            }
            QComboBox::down-arrow {
                image: url(:/images/icons/dropdown.svg);
                width: 12px;
                height: 12px;
                margin-right: 5px;
            }
            QComboBox QAbstractItemView {
                background-color: #2C3E50;
                color: white;
                selection-background-color: #3498DB;
            }
        """)
        self.topic_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        top_layout.addWidget(self.topic_combo, 3)
        
        # 创建刷新按钮
        self.refresh_btn = QPushButton("刷新")
        self.refresh_btn.setStyleSheet("""
            QPushButton {
                background-color: #2980B9;
                color: white;
                border-radius: 4px;
                padding: 5px;
                font-weight: bold;
                min-width: 70px;
                min-height: 25px;
            }
            QPushButton:hover {
                background-color: #3498DB;
            }
            QPushButton:pressed {
                background-color: #2472A4;
            }
        """)
        self.refresh_btn.clicked.connect(self.refresh_topics)
        top_layout.addWidget(self.refresh_btn, 1)
        
        # 创建打印/暂停按钮
        self.log_btn = QPushButton("打印")
        self.log_btn.setStyleSheet("""
            QPushButton {
                background-color: #27AE60;
                color: white;
                border-radius: 4px;
                padding: 5px;
                font-weight: bold;
                min-width: 70px;
                min-height: 25px;
            }
            QPushButton:hover {
                background-color: #2ECC71;
            }
            QPushButton:pressed {
                background-color: #219653;
            }
        """)
        self.log_btn.clicked.connect(self.toggle_logging)
        top_layout.addWidget(self.log_btn, 1)
        
        # 添加顶部控件到主布局
        main_layout.addWidget(top_widget)
        
        # 创建日志显示区域
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setLineWrapMode(QTextEdit.WidgetWidth)  # 启用自动换行
        self.log_text.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)  # 按需显示水平滚动条
        self.log_text.setStyleSheet("""
            QTextEdit {
                background-color: #1A202C;
                color: #F1F2F6;
                border: 1px solid #3498DB;
                border-radius: 4px;
                padding: 5px;
                font-family: Consolas, Monaco, Monospace;
                font-size: 10pt;
            }
            QScrollBar:vertical {
                background: #1A202C;
                width: 12px;
                margin: 0px 0px 0px 0px;
            }
            QScrollBar::handle:vertical {
                background: #3498DB;
                min-height: 20px;
                border-radius: 6px;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                border: none;
                background: none;
                height: 0px;
            }
        """)
        main_layout.addWidget(self.log_text)
        
        # 设置布局比例
        main_layout.setStretch(0, 0)  # 标题占用最小空间
        main_layout.setStretch(1, 0)  # 顶部控件占用最小空间
        main_layout.setStretch(2, 1)  # 日志显示区域占用所有剩余空间
        
        # 立即刷新话题列表
        self.refresh_topics()
        
    def monitor_topics(self):
        """后台线程，定期检查是否有新话题可用"""
        while self.running and not rospy.is_shutdown():
            try:
                # 每5秒检查一次话题列表
                time.sleep(5)
                
                # 获取当前发布的所有话题
                new_topics = self.get_all_topics()
                
                # 如果话题列表有变化，更新UI
                if set(new_topics) != set(self.topics_list):
                    self.topics_updated_signal.emit(new_topics)
            except Exception as e:
                rospy.logerr(f"监控话题时出错: {str(e)}")
                time.sleep(1)
                
    def get_all_topics(self):
        """获取当前所有可用的ROS话题"""
        try:
            # 获取所有话题和类型
            topics = rostopic.get_topic_list()
            topic_names = []
            
            # 过滤出话题名称
            for topic in topics[0]:
                # 只添加有类型的话题
                topic_name = topic[0]
                topic_names.append(topic_name)
                
            return sorted(topic_names)
        except Exception as e:
            rospy.logerr(f"获取话题列表时出错: {str(e)}")
            return []
            
    def refresh_topics(self):
        """刷新话题列表"""
        try:
            # 获取所有话题
            topics = self.get_all_topics()
            
            # 更新UI
            self.topics_updated_signal.emit(topics)
        except Exception as e:
            rospy.logerr(f"刷新话题列表时出错: {str(e)}")
            self.log_text.append(f"<span style='color:red'>刷新话题列表时出错: {str(e)}</span>")
            
    def update_topics_list(self, topics):
        """更新话题下拉框内容"""
        try:
            # 保存当前选择的话题
            current_selection = self.topic_combo.currentText()
            
            # 清空并更新话题列表
            self.topics_list = topics
            self.topic_combo.clear()
            self.topic_combo.addItems(self.topics_list)
            
            # 尝试恢复之前的选择
            index = self.topic_combo.findText(current_selection)
            if index >= 0:
                self.topic_combo.setCurrentIndex(index)
        except Exception as e:
            rospy.logerr(f"更新话题下拉框时出错: {str(e)}")
            
    def toggle_logging(self):
        """切换打印/暂停状态"""
        if not self.is_logging:
            # 开始打印
            self.start_logging()
        else:
            # 停止打印
            self.stop_logging()
            
    def start_logging(self):
        """开始打印选中话题的数据"""
        topic = self.topic_combo.currentText()
        
        if not topic:
            self.log_text.append("<span style='color:orange'>请选择一个话题</span>")
            return
            
        try:
            # 如果已有订阅，先取消
            if self.subscriber:
                self.subscriber.unregister()
                self.subscriber = None
                
            # 清空日志显示
            self.log_text.clear()
            self.log_text.append(f"<b>开始监控话题: {topic}</b>")
            
            # 获取话题类型
            topic_type = self.get_topic_type(topic)
            if not topic_type:
                self.log_text.append(f"<span style='color:red'>无法获取话题类型: {topic}</span>")
                return
                
            # 导入消息类型
            msg_class = rostopic.get_topic_class(topic)[0]
            if not msg_class:
                self.log_text.append(f"<span style='color:red'>无法获取话题消息类: {topic}</span>")
                return
                
            # 订阅话题
            self.current_topic = topic
            self.subscriber = rospy.Subscriber(
                topic,
                msg_class,
                self.message_callback,
                queue_size=10
            )
            
            # 更新按钮状态
            self.is_logging = True
            self.log_btn.setText("暂停")
            self.log_btn.setStyleSheet("""
                QPushButton {
                    background-color: #E74C3C;
                    color: white;
                    border-radius: 4px;
                    padding: 5px;
                    font-weight: bold;
                    min-width: 70px;
                    min-height: 25px;
                }
                QPushButton:hover {
                    background-color: #F75C4C;
                }
                QPushButton:pressed {
                    background-color: #D74C3C;
                }
            """)
        except Exception as e:
            self.log_text.append(f"<span style='color:red'>订阅话题时出错: {str(e)}</span>")
            rospy.logerr(f"订阅话题时出错: {str(e)}")
            
    def stop_logging(self):
        """停止打印话题数据"""
        try:
            # 取消订阅
            if self.subscriber:
                self.subscriber.unregister()
                self.subscriber = None
                
            # 更新按钮状态
            self.is_logging = False
            self.log_btn.setText("打印")
            self.log_btn.setStyleSheet("""
                QPushButton {
                    background-color: #27AE60;
                    color: white;
                    border-radius: 4px;
                    padding: 5px;
                    font-weight: bold;
                    min-width: 70px;
                    min-height: 25px;
                }
                QPushButton:hover {
                    background-color: #2ECC71;
                }
                QPushButton:pressed {
                    background-color: #219653;
                }
            """)
            
            # 清空日志显示
            self.log_text.clear()
            self.log_text.append("<b>已停止监控</b>")
        except Exception as e:
            self.log_text.append(f"<span style='color:red'>停止监控时出错: {str(e)}</span>")
            rospy.logerr(f"停止监控时出错: {str(e)}")
            
    def get_topic_type(self, topic):
        """获取话题的消息类型"""
        try:
            topic_type, _, _ = rostopic.get_topic_type(topic)
            return topic_type
        except Exception as e:
            rospy.logerr(f"获取话题类型时出错: {str(e)}")
            return None
            
    def message_callback(self, msg):
        """话题消息回调函数"""
        try:
            # 将消息对象转换为格式化文本
            text = self.format_message(msg)
            
            # 发送信号更新UI
            self.topic_message_signal.emit(text)
        except Exception as e:
            rospy.logerr(f"处理话题消息时出错: {str(e)}")
            
    def format_message(self, msg):
        """将ROS消息格式化为可读文本，模仿rostopic echo的输出格式"""
        try:
            # 添加时间戳标题
            timestamp = time.strftime("%H:%M:%S", time.localtime())
            result = [f"[{timestamp}]"]
            
            # 递归处理消息
            msg_text = self.format_ros_message(msg, indent_level=0)
            result.append(msg_text)
            result.append("-" * 30)  # 减少分隔线长度，避免导致换行
            
            return "\n".join(result)
        except Exception as e:
            return f"[格式化错误] {str(e)}\n原始消息: {str(msg)}"
    
    def format_ros_message(self, msg, indent_level=0):
        """递归格式化ROS消息对象，模仿rostopic echo的输出格式"""
        if msg is None:
            return "None"
            
        # 基本类型直接返回
        if isinstance(msg, (bool, int, float, str)):
            if isinstance(msg, float):
                # 处理浮点数，保留合适的精度
                if abs(msg) < 0.0001 and msg != 0:
                    return f"{msg:.10g}"  # 对于非常小的数值使用科学计数法
                else:
                    return f"{msg:.6g}"  # 对于普通数值保留6位有效数字
            return str(msg)
            
        indent = "  " * indent_level
        result = []
        
        if hasattr(msg, "__slots__"):
            # 处理ROS消息对象
            for slot in msg.__slots__:
                value = getattr(msg, slot)
                
                # 特殊处理covariance数组等大型数组
                if slot == "covariance" and isinstance(value, tuple) and len(value) > 10:
                    # 紧凑表示协方差矩阵
                    result.append(f"{indent}{slot}: [...]  # {len(value)}个元素")
                    continue
                    
                # 处理嵌套的ROS消息
                formatted_value = self.format_ros_message(value, indent_level + 1)
                result.append(f"{indent}{slot}: {formatted_value}")
                
        elif isinstance(msg, (list, tuple)):
            # 处理列表或元组
            if len(msg) > 10:
                # 大数组简化显示
                result.append(f"[...]  # {len(msg)}个元素")
            else:
                items = []
                for item in msg:
                    items.append(self.format_ros_message(item, indent_level))
                result.append(f"[{', '.join(items)}]")
                
        elif isinstance(msg, dict):
            # 处理字典
            if not msg:  # 空字典
                return "{}"
                
            result.append("{")
            for k, v in msg.items():
                formatted_value = self.format_ros_message(v, indent_level + 1)
                result.append(f"{indent}  {k}: {formatted_value}")
            result.append(f"{indent}}}")
            
        elif hasattr(msg, "tolist"):  # numpy arrays
            # 处理numpy数组
            try:
                array_list = msg.tolist()
                if len(array_list) > 10:
                    return f"[...]  # {len(array_list)}个元素"
                else:
                    return str(array_list)
            except:
                return str(msg)
        else:
            # 其他类型
            return str(msg)
            
        if indent_level == 0:
            # 顶层返回完整字符串
            return "\n".join(result)
        else:
            # 嵌套层次返回适当格式
            if len(result) == 1:
                return result[0].lstrip()
            else:
                return "\n" + "\n".join(result)
            
    def update_log_text(self, text):
        """更新日志显示文本"""
        try:
            # 添加新消息到文本框，使用monospace字体确保对齐，pre标签保留空格和格式
            formatted_text = f"<pre style='font-family:Consolas,Monaco,Monospace;white-space:pre;'>{text}</pre>"
            self.log_text.append(formatted_text)
            
            # 滚动到底部
            scrollbar = self.log_text.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())
            
            # 限制日志长度，避免内存占用过多
            document = self.log_text.document()
            if document.blockCount() > 1000:  # 限制最多1000行
                cursor = self.log_text.textCursor()
                cursor.movePosition(QTextCursor.Start)
                cursor.movePosition(QTextCursor.Down, QTextCursor.KeepAnchor, document.blockCount() - 800)  # 保留800行
                cursor.removeSelectedText()
        except Exception as e:
            rospy.logerr(f"更新日志显示时出错: {str(e)}")
            
    def shutdown(self):
        """关闭组件，清理资源"""
        # 避免重复关闭
        if self.is_shutting_down:
            return
            
        self.is_shutting_down = True
        
        # 首先停止记录
        if self.is_logging:
            try:
                self.stop_logging()
            except Exception as e:
                rospy.logerr(f"停止日志记录时出错: {str(e)}")
        
        # 停止后台线程
        self.running = False
        
        # 等待线程结束
        try:
            if hasattr(self, 'topic_monitor_thread') and self.topic_monitor_thread.is_alive():
                self.topic_monitor_thread.join(1.0)
        except Exception as e:
            rospy.logerr(f"停止监控线程时出错: {str(e)}")
            
        # 断开信号连接
        try:
            self.topic_message_signal.disconnect()
            self.topics_updated_signal.disconnect()
        except Exception:
            # 可能已经没有连接，忽略异常
            pass
            
        # 取消话题订阅
        if self.subscriber:
            try:
                self.subscriber.unregister()
                self.subscriber = None
            except Exception as e:
                rospy.logerr(f"取消话题订阅时出错: {str(e)}")

if __name__ == "__main__":
    # 独立运行测试
    import sys
    
    app = QApplication(sys.argv)
    
    # 初始化ROS节点
    rospy.init_node('topic_logger_test', anonymous=True)
    
    # 创建组件
    logger = TopicLogger()
    logger.setMinimumSize(600, 400)
    logger.show()
    
    # 启动Qt事件循环
    sys.exit(app.exec_()) 