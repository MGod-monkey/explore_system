#!/usr/bin/env python
# -*- coding: utf-8 -*-

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

import math

class DashBoard(QWidget):
    """速度表盘组件，用于显示无人机速度"""
    
    def __init__(self, parent=None):
        QWidget.__init__(self, parent)
        
        # 保存父组件引用
        self.parent = parent
        
        # 初始化数值
        self._gear = 4
        self._rpm = 0
        self._speed = 0
        self._temperature = 0
        self._oil = 0
        
        # 设置背景透明
        self.setAttribute(Qt.WA_TranslucentBackground)
        
        # 设置尺寸策略，使组件能够根据父容器调整大小
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # 设置最小尺寸，确保表盘能够完整显示
        self.setMinimumSize(300, 300)
        
        # 调整大小
        self.resize(parent.size() if parent else QSize(300, 300))
    
    def set_gear(self, gear):
        """设置挡位"""
        self._gear = gear
        self.update()
    
    def set_rpm(self, rpm):
        """设置转速"""
        self._rpm = rpm
        self.update()
    
    def set_speed(self, speed):
        """设置速度值(cm/s)"""
        self._speed = speed
        self._rpm = speed  # 在这个实现中，转速表示速度
        self.update()
    
    def set_temperature(self, temperature):
        """设置温度值"""
        self._temperature = temperature
        self.update()
    
    def set_oil(self, oil):
        """设置油量百分比"""
        self._oil = oil
        self.update()
    
    def sizeHint(self):
        """返回推荐尺寸"""
        return QSize(350, 350)
    
    def minimumSizeHint(self):
        """返回最小推荐尺寸"""
        return QSize(300, 300)
    
    def paintEvent(self, event):
        """绘制表盘"""
        # 根据父组件大小调整
        if self.parent:
            width = self.parent.width()
            height = self.parent.height()
            self.resize(width, height)
        
        QWidget.paintEvent(self, event)
        
        # 计算合适的缩放尺寸和中心点
        width = self.width()
        height = self.height()
        side = min(width, height)
        center_x = width / 2
        center_y = height / 2
        
        # 创建画布
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.translate(center_x, center_y)
        painter.scale(side / 200.0, side / 200.0)
        painter.setPen(Qt.NoPen)
        painter.setBrush(Qt.NoBrush)
        
        # 绘制各部分
        self.draw_tachometer(painter)
        self.draw_speedometer(painter)
        self.draw_gear(painter)
        
    def draw_tachometer(self, painter):
        """绘制转速表(速度表)"""
        # 定义颜色
        # normal_color = QColor(18, 11, 10, 245)
        normal_color = QColor(237, 245, 246, 245)
        overrun_color = QColor(245, 64, 64, 225)
        
        # 绘制表盘外檐
        painter.save()
        painter.setPen(QPen(normal_color, 1, Qt.SolidLine))
        rect = QRect(-95, -95, 190, 190)
        painter.drawArc(rect, 0, 270 * 16)
        painter.restore()
        
        # 绘制红色区域
        painter.save()
        rectangle_outer = QRectF(-95, -95, 190, 190)
        rectangle_inner = QRectF(-87, -87, 174, 174)
        painter.setBrush(overrun_color)
        path = QPainterPath()
        path.arcTo(rectangle_outer, 0.0, 108.0)
        path.arcTo(rectangle_inner, 108, -108)
        painter.drawPath(path)
        painter.restore()
        
        # 绘制大刻度
        painter.save()
        painter.setPen(QPen(normal_color, 1, Qt.SolidLine))
        painter.rotate(90)
        for i in range(21):
            painter.drawLine(88, 0, 94, 0)
            painter.rotate(13.5)
        painter.restore()
        
        # 绘制小刻度
        painter.save()
        painter.setPen(QPen(normal_color, 1, Qt.SolidLine))
        painter.rotate(90)
        for i in range(100):
            painter.drawLine(91, 0, 94, 0)
            painter.rotate(2.7)
        painter.restore()
        
        # 绘制表盘数字
        painter.save()
        painter.rotate(90)
        painter.setPen(normal_color)
        painter.setFont(QFont("Arial", 14))
        for i in range(11):
            painter.save()
            if i > 6:
                painter.setPen(overrun_color)
            painter.rotate(27.0 * i)
            painter.translate(76, 0)
            painter.rotate(270 - 27.0 * i)
            painter.drawText(QRect(-20, -10, 40, 20), Qt.AlignCenter, str(i))
            painter.restore()
        painter.restore()
        
        # 绘制指针
        # 使用QPolygon而不是Python列表
        hand_points = [QPoint(-4, 0), QPoint(0, 94), QPoint(4, 0), QPoint(0, -6)]
        hand_polygon = QPolygon(hand_points)
        hand_color = QColor(0x88, 0x37, 0x4f, 176)
        
        painter.save()
        painter.setPen(Qt.NoPen)
        painter.setBrush(hand_color)
        painter.rotate(27.0 * (self._rpm / 10.0))
        painter.drawConvexPolygon(hand_polygon)
        painter.restore()
        
        # 绘制文字
        painter.save()
        painter.setPen(normal_color)
        painter.setFont(QFont("Arial", 8))
        painter.drawText(QRect(-50, -70, 100, 50), Qt.AlignCenter, "×10")
        painter.setFont(QFont("Arial", 8, 50, True))
        painter.drawText(QRect(-50, 34, 32, 16), Qt.AlignCenter, "CM/S")
        painter.restore()
    
    def draw_speedometer(self, painter):
        """绘制数字速度表"""
        painter.save()
        
        painter.setPen(QColor(64, 64, 245))
        painter.setFont(QFont("Arial", 6, 50, True))
        painter.drawText(QRect(60, 50, 70, 20), Qt.AlignCenter, "SPEED")
        
        painter.setPen(QColor(26, 245, 245))
        painter.setFont(QFont("Arial", 24, 63, True))
        painter.drawText(QRect(60, 48, 70, 50), Qt.AlignBottom | Qt.AlignLeft,
                        "{:03d}".format(self._speed))
        
        painter.setPen(QColor(26, 245, 245))
        painter.setFont(QFont("Arial", 8, 63, True))
        painter.drawText(QRect(125, 75, 40, 20), Qt.AlignBottom | Qt.AlignLeft,
                        "cm/s")
        
        painter.restore()
    
    def draw_gear(self, painter):
        """绘制挡位显示"""
        gear_rect = QRect(0, 0, 80, 80)
        suffix_rect = QRect(48, 48, 32, 32)
        suffix_font = QFont("Arial", 16, 63, True)
        
        painter.save()
        painter.setPen(QPen(QColor(26, 245, 245), 1, Qt.SolidLine))
        painter.setFont(QFont("Arial", 48, 63, True))
        
        # 根据挡位显示不同文字
        if self._gear == 1:
            painter.drawText(gear_rect, Qt.AlignCenter, str(self._gear))
            painter.setFont(suffix_font)
            painter.drawText(suffix_rect, Qt.AlignCenter, "st")
        elif self._gear == 2:
            painter.drawText(gear_rect, Qt.AlignCenter, str(self._gear))
            painter.setFont(suffix_font)
            painter.drawText(suffix_rect, Qt.AlignCenter, "nd")
        elif self._gear == 3:
            painter.drawText(gear_rect, Qt.AlignCenter, str(self._gear))
            painter.setFont(suffix_font)
            painter.drawText(suffix_rect, Qt.AlignCenter, "rd")
        elif 4 <= self._gear <= 8:
            painter.drawText(gear_rect, Qt.AlignCenter, str(self._gear))
            painter.setFont(suffix_font)
            painter.drawText(suffix_rect, Qt.AlignCenter, "th")
        elif self._gear == 10:  # D
            painter.drawText(gear_rect, Qt.AlignCenter, "D")
        elif self._gear == 11:  # N
            painter.drawText(gear_rect, Qt.AlignCenter, "N")
        elif self._gear == 12:  # P
            painter.drawText(gear_rect, Qt.AlignCenter, "P")
        elif self._gear == 13:  # R
            painter.drawText(gear_rect, Qt.AlignCenter, "R")
        
        painter.restore() 