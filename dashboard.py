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


class AttitudeIndicator(QWidget):
    """无人机姿态指示器组件，显示俯仰角和滚转角"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 加载图像资源
        self.bg_image = QPixmap(":/images/icons/attitudebg1.png")
        self.fg_image = QPixmap(":/images/icons/attitudefg2.png")
        
        # 设置当前姿态参数
        self.pitch = 0
        self.roll = 0
        
        # 设置最小大小
        self.setMinimumSize(150, 150)
        # self.setMaximumSize(200, 200)
        
        # 设置背景透明
        self.setAttribute(Qt.WA_TranslucentBackground)
        
    def pitch_to_pixels(self, pitch):
        # 线性映射: -90..90俯仰角度映射到-262..262像素
        max_pitch = 90
        max_pixels = 262
        return (pitch / max_pitch) * max_pixels
        
    def paintEvent(self, event):
        """绘制姿态指示器
        
        参数:
            event: 绘图事件
        """
        # 创建绘图器
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)  # 抗锯齿
        painter.setRenderHint(QPainter.SmoothPixmapTransform)  # 平滑图像变换
        
        # 获取组件尺寸
        width = self.width()
        height = self.height()
        
        # 计算缩放系数
        size = min(width, height)
        scale_factor = size / 300.0  # 基于300px的设计尺寸计算缩放
        
        # 计算中心点
        center_x = width / 2
        center_y = height / 2
        
        # 设置背景颜色 - 使用不透明的深色背景
        painter.fillRect(0, 0, width, height, QColor(30, 35, 48))  # 深蓝色背景，与系统风格匹配
        
        # 如果没有加载图像，先绘制提示文本
        if self.bg_image is None or self.fg_image is None:
            # 绘制提示文本
            painter.setPen(Qt.white)
            painter.drawText(event.rect(), Qt.AlignCenter, "姿态指示器\n图像资源正在加载...")
            return
            
        # 获取当前姿态角度值
        pitch = self.pitch
        roll = self.roll
        
        # 初始化变量
        flip = False
        effective_roll = roll  # 默认使用原始滚转角
        
        # 处理跨越90°俯仰角的边缘情况
        if pitch > 90:
            pitch = 180 - pitch
            flip = True
            effective_roll = -self.roll + 180
        elif pitch < -90:
            pitch = -180 - pitch
            flip = True
            effective_roll = -self.roll - 180

        # 将滚转角归一化到-180到180范围内
        while effective_roll > 180:
            effective_roll -= 360
        while effective_roll < -180:
            effective_roll += 360
            
        pitch_offset = self.pitch_to_pixels(pitch)
        
        # 计算基于有效滚转角的偏移方向
        roll_radians = math.radians(effective_roll)
        dx = -pitch_offset * math.sin(roll_radians)
        dy = pitch_offset * math.cos(roll_radians)
        
        # 绘制背景图像（地平线）
        bg_pixmap = self.bg_image
        
        # 处理翻转
        transform = QTransform()
        if flip:
            transform.rotate(180)
        
        transform.rotate(-effective_roll)
        
        # 应用缩放
        transform.scale(scale_factor, scale_factor)
        
        bg_rotated = bg_pixmap.transformed(transform, Qt.SmoothTransformation)
        
        # 绘制背景（人工地平线）
        target_x = int(center_x - bg_rotated.width()/2 + dx * scale_factor)
        target_y = int(center_y - bg_rotated.height()/2 + dy * scale_factor)
        painter.drawPixmap(target_x, target_y, bg_rotated)
        
        # 绘制前景（固定部分）
        fg_scaled = self.fg_image.scaled(
            int(self.fg_image.width() * scale_factor), 
            int(self.fg_image.height() * scale_factor), 
            Qt.KeepAspectRatio, 
            Qt.SmoothTransformation
        )
        fg_x = int(center_x - fg_scaled.width()/2)
        fg_y = int(center_y - fg_scaled.height()/2)
        painter.drawPixmap(fg_x, fg_y, fg_scaled)
        
    def update_attitude(self, pitch, roll):
        """更新姿态指示器的俯仰和滚转角度
        
        参数:
            pitch (float): 俯仰角度，正值表示上仰，负值表示下俯
            roll (float): 滚转角度，正值表示右转，负值表示左转
        """
        self.pitch = pitch
        self.roll = roll
        self.update()  # 触发重绘
    
    def sizeHint(self):
        """返回组件的建议大小"""
        return QSize(350, 350)  # 增大默认尺寸以适应仪表盘区域


class UIButton(QWidget):
    """扇形控制按钮组件，用于控制中心"""
    
    # 信号定义
    centerClicked = pyqtSignal()  # 中央按钮点击信号
    topClicked = pyqtSignal()     # 上方按钮点击信号
    rightClicked = pyqtSignal()   # 右侧按钮点击信号
    leftClicked = pyqtSignal()    # 左侧按钮点击信号
    bottomClicked = pyqtSignal()  # 底部按钮点击信号
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 设置最小大小
        self.setMinimumSize(300, 300)
        
        # 设置尺寸策略，允许组件根据父容器调整大小
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # 扇形尺寸参数
        self.outerPieRadius = 180  # 增大外圆半径
        self.innerPieRadius = 80   # 增大内圆半径，增加与中间按钮的间距
        
        # 按钮状态变量
        self.mouseCenterView = False
        self.mouseBottomView = False
        self.mouseRightView = False
        self.mouseTopView = False
        self.mouseLeftView = False
        
        # 设置鼠标跟踪
        self.setMouseTracking(True)
        
        # 初始化路径变量，用于存储各个区域的路径
        self.centerBtnView = None
        self.topBtnView = None
        self.rightBtnView = None
        self.leftBtnView = None
        self.bottomBtnView = None
        
        # 设置文本
        self.centerText = "一键启动"  # 中间按钮现在使用图标，文本作为工具提示显示
        # self.topText = "一键返航"
        self.rightText = "停止程序"
        self.leftText = "开始探索"
        # self.bottomText = "功能待定"
        
        # 设置颜色 - 使用与系统主题匹配的配色方案
        self.centerColor = QColor("#27AE60")  # 绿色，启动按钮
        self.topColor = QColor("#8E44AD")     # 紫色，返航按钮
        self.rightColor = QColor("#E74C3C")   # 红色，停止按钮
        self.leftColor = QColor("#2980B9")    # 蓝色，探索按钮
        self.bottomColor = QColor("#7F8C8D")  # 灰色，待定按钮
        
        # 加载中间按钮图标
        self.centerIcon = QPixmap(":/images/icons/start.svg")
        
        # 设置工具提示样式
        self.setStyleSheet("""
            QToolTip {
                background-color: #2C3E50;
                color: #FFFFFF;
                border: 1px solid #3498DB;
                border-radius: 4px;
                padding: 4px;
                font-size: 11pt;
                opacity: 225;
            }
        """)

    def drawOuterPie(self, painter):
        """绘制右侧按钮"""
        painter.save()
        # 设置标志位，判断鼠标是否进入该区域
        if self.mouseRightView:
            radius1 = self.outerPieRadius + 4
        else:
            radius1 = self.outerPieRadius

        # 绘制大扇形
        rect = QRectF(-radius1/2, -radius1/2, radius1, radius1)
        pathOuterChampagnePie = QPainterPath()
        pathOuterChampagnePie.arcMoveTo(rect, -45)
        pathOuterChampagnePie.arcTo(rect, -45, 90)
        pathOuterChampagnePie.lineTo(0, 0)
        pathOuterChampagnePie.closeSubpath()
        
        # 绘制小扇形
        radius = self.innerPieRadius
        rect1 = QRectF(-radius/2, -radius/2, radius, radius)
        pathMidPie = QPainterPath()
        pathMidPie.arcMoveTo(rect1, -45)
        pathMidPie.arcTo(rect1, -45, 90)
        pathMidPie.lineTo(0, 0)
        pathMidPie.closeSubpath()

        # 大扇形减去小扇形，得到扇形饼圆
        self.rightBtnView = pathOuterChampagnePie.subtracted(pathMidPie)
        
        # 设置文字路径
        textPath = QPainterPath()
        # 使用更小的字体
        font = QFont("WenQuanYi Micro Hei", 8)
        # 位置调整，使文字在扇形中央
        textX = radius1 * 0.24
        textY = radius1 * 0.05
        textPath.addText(textX, textY, font, self.rightText)
        
        # 绘制图形和文字
        painter.setPen(Qt.NoPen)
        painter.setBrush(self.rightColor)
        painter.drawPath(self.rightBtnView)
        
        # 绘制文字
        painter.setPen(Qt.white)
        painter.drawPath(textPath)
        
        painter.restore()

    # def drawOuterCircle(self, painter):
    #     """绘制顶部按钮"""
    #     painter.save()
    #     # 设置标志位，判断鼠标是否进入该区域
    #     if self.mouseTopView:
    #         radius1 = self.outerPieRadius + 4
    #     else:
    #         radius1 = self.outerPieRadius

    #     # 绘制大扇形
    #     rect = QRectF(-radius1/2, -radius1/2, radius1, radius1)
    #     pathOuterChampagnePie = QPainterPath()
    #     pathOuterChampagnePie.arcMoveTo(rect, 45)
    #     pathOuterChampagnePie.arcTo(rect, 45, 90)
    #     pathOuterChampagnePie.lineTo(0, 0)
    #     pathOuterChampagnePie.closeSubpath()
        
    #     # 设置文字路径
    #     textPath = QPainterPath()
    #     # 使用更小的字体
    #     font = QFont("WenQuanYi Micro Hei", 8)
    #     # 位置调整，使文字在扇形中央
    #     textX = -radius1 * 0.12
    #     textY = -radius1 * 0.3
    #     textPath.addText(textX, textY, font, self.topText)
 
    #     # 绘制小扇形
    #     radius = self.innerPieRadius
    #     rect1 = QRectF(-radius/2, -radius/2, radius, radius)
    #     pathMidPie = QPainterPath()
    #     pathMidPie.arcMoveTo(rect1, 45)
    #     pathMidPie.arcTo(rect1, 45, 90)
    #     pathMidPie.lineTo(0, 0)
    #     pathMidPie.closeSubpath()

    #     # 大扇形减去小扇形，得到扇形饼圆
    #     self.topBtnView = pathOuterChampagnePie.subtracted(pathMidPie)
        
    #     # 绘制图形和文字
    #     painter.setPen(Qt.NoPen)
    #     painter.setBrush(self.topColor)
    #     painter.drawPath(self.topBtnView)
        
    #     # 绘制文字
    #     painter.setPen(Qt.black)
    #     painter.drawPath(textPath)
        
    #     painter.restore()

    def drawInnerPie(self, painter):
        """绘制左侧按钮"""
        painter.save()
        # 设置标志位，判断鼠标是否进入该区域
        if self.mouseLeftView:
            radius1 = self.outerPieRadius + 4
        else:
            radius1 = self.outerPieRadius

        # 绘制大扇形
        rect = QRectF(-radius1/2, -radius1/2, radius1, radius1)
        pathOuterChampagnePie = QPainterPath()
        pathOuterChampagnePie.arcMoveTo(rect, 135)
        pathOuterChampagnePie.arcTo(rect, 135, 90)
        pathOuterChampagnePie.lineTo(0, 0)
        pathOuterChampagnePie.closeSubpath()
        
        # 设置文字路径
        textPath = QPainterPath()
        # 使用更小的字体
        font = QFont("WenQuanYi Micro Hei", 8)
        # 位置调整，使文字在扇形中央
        textX = -radius1 * 0.48
        textY = radius1 * 0.05
        textPath.addText(textX, textY, font, self.leftText)
      
        # 绘制小扇形
        radius = self.innerPieRadius
        rect1 = QRectF(-radius/2, -radius/2, radius, radius)
        pathMidPie = QPainterPath()
        pathMidPie.arcMoveTo(rect1, 135)
        pathMidPie.arcTo(rect1, 135, 90)
        pathMidPie.lineTo(0, 0)
        pathMidPie.closeSubpath()

        # 大扇形减去小扇形，得到扇形饼圆
        self.leftBtnView = pathOuterChampagnePie.subtracted(pathMidPie)
        
        # 绘制图形和文字
        painter.setPen(Qt.NoPen)
        painter.setBrush(self.leftColor)
        painter.drawPath(self.leftBtnView)
        
        # 绘制文字
        painter.setPen(Qt.white)
        painter.drawPath(textPath)
        
        painter.restore()

    # def drawBottom(self, painter):
    #     """绘制底部按钮"""
    #     painter.save()
    #     # 设置标志位，判断鼠标是否进入该区域
    #     if self.mouseBottomView:
    #         radius1 = self.outerPieRadius + 4
    #     else:
    #         radius1 = self.outerPieRadius

    #     # 绘制大扇形
    #     rect = QRectF(-radius1/2, -radius1/2, radius1, radius1)
    #     pathOuterChampagnePie = QPainterPath()
    #     pathOuterChampagnePie.arcMoveTo(rect, 225)
    #     pathOuterChampagnePie.arcTo(rect, 225, 90)
    #     pathOuterChampagnePie.lineTo(0, 0)
    #     pathOuterChampagnePie.closeSubpath()
        
    #     # 设置文字路径
    #     textPath = QPainterPath()
    #     # 使用更小的字体
    #     font = QFont("WenQuanYi Micro Hei", 8)
    #     # 位置调整，使文字在扇形中央
    #     textX = -radius1 * 0.12
    #     textY = radius1 * 0.35
    #     textPath.addText(textX, textY, font, self.bottomText)

    #     # 绘制小扇形
    #     radius = self.innerPieRadius
    #     rect1 = QRectF(-radius/2, -radius/2, radius, radius)
    #     pathMidPie = QPainterPath()
    #     pathMidPie.arcMoveTo(rect1, 225)
    #     pathMidPie.arcTo(rect1, 225, 90)
    #     pathMidPie.lineTo(0, 0)
    #     pathMidPie.closeSubpath()

    #     # 大扇形减去小扇形，得到扇形饼圆
    #     self.bottomBtnView = pathOuterChampagnePie.subtracted(pathMidPie)
        
    #     # 绘制图形和文字
    #     painter.setPen(Qt.NoPen)
    #     painter.setBrush(self.bottomColor)
    #     painter.drawPath(self.bottomBtnView)
        
    #     # 绘制文字
    #     painter.setPen(Qt.white)
    #     painter.drawPath(textPath)
        
    #     painter.restore()

    def drawMidCircle(self, painter):
        """绘制中间按钮"""
        if self.mouseCenterView:
            radius = 40 + 4
        else:
            radius = 40
        
        painter.save()
        painter.setPen(Qt.NoPen)
        painter.setBrush(self.centerColor)
        
        # 创建圆形路径
        path = QPainterPath()
        path.addEllipse(-radius/2, -radius/2, radius, radius)
        
        # 绘制圆形
        painter.drawPath(path)
        self.centerBtnView = path
        
        # 绘制图标 (使用起始按钮图标)
        if self.centerIcon:
            # 缩放图标以适应按钮大小
            iconSize = min(radius - 8, 32)  # 图标大小限制
            scaledIcon = self.centerIcon.scaled(iconSize, iconSize, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            
            # 计算图标位置
            iconX = -scaledIcon.width() / 2
            iconY = -scaledIcon.height() / 2
            
            # 创建白色图标
            coloredIcon = QPixmap(scaledIcon.size())
            coloredIcon.fill(Qt.transparent)
            iconPainter = QPainter(coloredIcon)
            iconPainter.setRenderHint(QPainter.Antialiasing)
            iconPainter.setRenderHint(QPainter.SmoothPixmapTransform)
            iconPainter.setCompositionMode(QPainter.CompositionMode_SourceOver)
            iconPainter.drawPixmap(0, 0, scaledIcon)
            iconPainter.setCompositionMode(QPainter.CompositionMode_SourceIn)
            iconPainter.fillRect(coloredIcon.rect(), QColor("#FFFFFF"))  # 白色图标
            iconPainter.end()
            
            # 绘制处理后的图标
            painter.drawPixmap(int(iconX), int(iconY), coloredIcon)
        
        painter.restore()

    def resizeEvent(self, event):
        """重写尺寸变更事件处理"""
        super().resizeEvent(event)
        # 不再动态调整按钮大小，保持原有比例

    def paintEvent(self, event):
        """绘制组件"""
        # 绘制准备工作, 启用反锯齿
        painter = QPainter(self)
        painter.setRenderHints(QPainter.Antialiasing | QPainter.TextAntialiasing)

        # 平移坐标中心，等比例缩放
        width = self.width()
        height = self.height()
        side = min(width, height)
        painter.translate(width/2, height/2)  # 坐标中心移至窗口中心
        painter.scale(side/200.0, side/200.0)  # 坐标刻度缩放

        # 绘制各个部分
        # self.drawOuterCircle(painter)  # 绘制顶部按钮
        self.drawOuterPie(painter)     # 绘制右侧按钮
        self.drawInnerPie(painter)     # 绘制左侧按钮
        # self.drawBottom(painter)       # 绘制底部按钮
        self.drawMidCircle(painter)    # 绘制中间按钮

    def mouseMoveEvent(self, event):
        """鼠标移动事件处理"""
        # 坐标系转换
        width = self.width()
        height = self.height()
        side = min(width, height)
        enterPoint = QPoint((event.pos().x() - width/2) / (side/200.0),
                           (event.pos().y() - height/2) / (side/200.0))

        # 判断鼠标是否进入各个区域，并设置标志位
        # 中间按钮
        if self.centerBtnView and self.centerBtnView.contains(enterPoint):
            if not self.mouseCenterView:
                self.mouseCenterView = True
                self.setCursor(Qt.PointingHandCursor)
                self.setToolTip(self.centerText)
                self.update()
        else:
            if self.mouseCenterView:
                self.mouseCenterView = False
                self.setCursor(Qt.ArrowCursor)
                self.update()

        # # 底部按钮
        # if self.bottomBtnView and self.bottomBtnView.contains(enterPoint):
        #     if not self.mouseBottomView:
        #         self.mouseBottomView = True
        #         self.setCursor(Qt.PointingHandCursor)
        #         self.setToolTip(self.bottomText)
        #         self.update()
        # else:
        #     if self.mouseBottomView:
        #         self.mouseBottomView = False
        #         self.setCursor(Qt.ArrowCursor)
        #         self.update()

        # 右侧按钮
        if self.rightBtnView and self.rightBtnView.contains(enterPoint):
            if not self.mouseRightView:
                self.mouseRightView = True
                self.setCursor(Qt.PointingHandCursor)
                self.setToolTip(self.rightText)
                self.update()
        else:
            if self.mouseRightView:
                self.mouseRightView = False
                self.setCursor(Qt.ArrowCursor)
                self.update()

        # # 顶部按钮
        # if self.topBtnView and self.topBtnView.contains(enterPoint):
        #     if not self.mouseTopView:
        #         self.mouseTopView = True
        #         self.setCursor(Qt.PointingHandCursor)
        #         self.setToolTip(self.topText)
        #         self.update()
        # else:
        #     if self.mouseTopView:
        #         self.mouseTopView = False
        #         self.setCursor(Qt.ArrowCursor)
        #         self.update()

        # 左侧按钮
        if self.leftBtnView and self.leftBtnView.contains(enterPoint):
            if not self.mouseLeftView:
                self.mouseLeftView = True
                self.setCursor(Qt.PointingHandCursor)
                self.setToolTip(self.leftText)
                self.update()
        else:
            if self.mouseLeftView:
                self.mouseLeftView = False
                self.setCursor(Qt.ArrowCursor)
                self.update()

    def mousePressEvent(self, event):
        """鼠标点击事件处理"""
        # 坐标系转换
        width = self.width()
        height = self.height()
        side = min(width, height)
        clickPoint = QPoint((event.pos().x() - width/2) / (side/200.0),
                           (event.pos().y() - height/2) / (side/200.0))

        # 判断点击位置并发射对应信号
        if self.centerBtnView and self.centerBtnView.contains(clickPoint):
            self.centerClicked.emit()

        # if self.bottomBtnView and self.bottomBtnView.contains(clickPoint):
        #     self.bottomClicked.emit()

        if self.rightBtnView and self.rightBtnView.contains(clickPoint):
            self.rightClicked.emit()

        # if self.topBtnView and self.topBtnView.contains(clickPoint):
        #     self.topClicked.emit()

        if self.leftBtnView and self.leftBtnView.contains(clickPoint):
            self.leftClicked.emit() 