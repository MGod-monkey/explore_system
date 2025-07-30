#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
手动控制器模块
参考手势识别控制器，实现软件界面按钮的无人机控制功能
"""

import rospy
import threading
import time
from mavros_msgs.msg import RCIn

# 全局配置参数
CONTROL_STEP = 400  # RC控制步长
DEFAULT_DURATION = 0.5  # 默认控制持续时间（秒）
CONTROL_RATE = 50  # 控制循环频率（Hz）

class ManualRCController:
    """手动RC控制器类"""
    
    def __init__(self):
        """初始化控制器"""
        # RC通道默认值（中位值1500）
        self.rc_values = [1500] * 18
        
        # 控制状态
        self.is_command_active = False
        self.last_command_time = 0
        self.command_duration = DEFAULT_DURATION
        self.current_command = None
        
        # 持续控制状态
        self.continuous_commands = {
            'forward': False,
            'backward': False,
            'left': False,
            'right': False,
            'up': False,
            'down': False
        }
        
        # 线程锁
        self.command_lock = threading.Lock()
        
        # 发布RC覆写
        self.rc_pub = rospy.Publisher('/mavros/rc/override', RCIn, queue_size=10)
        
        # 启动控制循环
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        print("手动RC控制器已初始化")

    def send_rc_command(self):
        """发送RC命令"""
        msg = RCIn()
        msg.header.stamp = rospy.Time.now()
        msg.channels = self.rc_values
        msg.rssi = 43
        self.rc_pub.publish(msg)

    def reset_channels(self):
        """重置所有通道到中位值"""
        with self.command_lock:
            self.rc_values = [1500] * 18
            self.is_command_active = False
            self.current_command = None
        self.send_rc_command()

    def execute_single_command(self, command_type, duration=None):
        """执行单次命令"""
        if duration is None:
            duration = DEFAULT_DURATION
            
        with self.command_lock:
            # 重置所有通道
            self.rc_values = [1500] * 18
            
            # 根据命令类型设置对应通道值
            if command_type == 'forward':
                self.rc_values[1] = 1500 + CONTROL_STEP  # 前进
            elif command_type == 'backward':
                self.rc_values[1] = 1500 - CONTROL_STEP  # 后退
            elif command_type == 'left':
                self.rc_values[0] = 1500 - CONTROL_STEP  # 左移
            elif command_type == 'right':
                self.rc_values[0] = 1500 + CONTROL_STEP  # 右移
            elif command_type == 'up':
                self.rc_values[2] = 1500 + CONTROL_STEP  # 上升
            elif command_type == 'down':
                self.rc_values[2] = 1500 - CONTROL_STEP  # 下降
            else:
                print(f"未知命令类型: {command_type}")
                return
            
            # 限制值在有效范围内
            for i in range(len(self.rc_values)):
                self.rc_values[i] = max(1000, min(self.rc_values[i], 2000))
            
            # 设置命令状态
            self.is_command_active = True
            self.last_command_time = time.time()
            self.command_duration = duration
            self.current_command = command_type
            
        self.send_rc_command()
        print(f"执行单次命令: {command_type}, 持续时间: {duration}s")

    def start_continuous_command(self, command_type):
        """开始持续命令"""
        with self.command_lock:
            # 停止其他持续命令
            for cmd in self.continuous_commands:
                self.continuous_commands[cmd] = False
            
            # 启动指定命令
            self.continuous_commands[command_type] = True
            
            # 重置所有通道
            self.rc_values = [1500] * 18
            
            # 根据命令类型设置对应通道值
            if command_type == 'forward':
                self.rc_values[1] = 1500 + CONTROL_STEP
            elif command_type == 'backward':
                self.rc_values[1] = 1500 - CONTROL_STEP
            elif command_type == 'left':
                self.rc_values[0] = 1500 - CONTROL_STEP
            elif command_type == 'right':
                self.rc_values[0] = 1500 + CONTROL_STEP
            elif command_type == 'up':
                self.rc_values[2] = 1500 + CONTROL_STEP
            elif command_type == 'down':
                self.rc_values[2] = 1500 - CONTROL_STEP
            else:
                print(f"未知持续命令类型: {command_type}")
                return
            
            # 限制值在有效范围内
            for i in range(len(self.rc_values)):
                self.rc_values[i] = max(1000, min(self.rc_values[i], 2000))
        
        self.send_rc_command()
        print(f"开始持续命令: {command_type}")

    def stop_continuous_command(self, command_type):
        """停止持续命令"""
        with self.command_lock:
            if command_type in self.continuous_commands:
                self.continuous_commands[command_type] = False
                
                # 检查是否还有其他持续命令在执行
                any_active = any(self.continuous_commands.values())
                if not any_active:
                    # 如果没有持续命令，重置所有通道
                    self.rc_values = [1500] * 18
        
        self.send_rc_command()
        print(f"停止持续命令: {command_type}")

    def stop_all_commands(self):
        """停止所有命令"""
        with self.command_lock:
            # 停止所有持续命令
            for cmd in self.continuous_commands:
                self.continuous_commands[cmd] = False
            
            # 停止单次命令
            self.is_command_active = False
            self.current_command = None
            
            # 重置所有通道
            self.rc_values = [1500] * 18
        
        self.send_rc_command()
        print("停止所有命令")

    def control_loop(self):
        """控制循环"""
        rate = rospy.Rate(CONTROL_RATE)
        
        while not rospy.is_shutdown():
            try:
                with self.command_lock:
                    # 检查单次命令是否超时
                    if self.is_command_active:
                        if time.time() - self.last_command_time >= self.command_duration:
                            self.is_command_active = False
                            self.current_command = None
                            # 如果没有持续命令在执行，重置通道
                            if not any(self.continuous_commands.values()):
                                self.rc_values = [1500] * 18
                    
                    # 处理持续命令
                    any_continuous_active = any(self.continuous_commands.values())
                    if any_continuous_active and not self.is_command_active:
                        # 重置所有通道
                        self.rc_values = [1500] * 18
                        
                        # 应用所有活跃的持续命令
                        for cmd_type, is_active in self.continuous_commands.items():
                            if is_active:
                                if cmd_type == 'forward':
                                    self.rc_values[1] = 1500 + CONTROL_STEP
                                elif cmd_type == 'backward':
                                    self.rc_values[1] = 1500 - CONTROL_STEP
                                elif cmd_type == 'left':
                                    self.rc_values[0] = 1500 - CONTROL_STEP
                                elif cmd_type == 'right':
                                    self.rc_values[0] = 1500 + CONTROL_STEP
                                elif cmd_type == 'up':
                                    self.rc_values[2] = 1500 + CONTROL_STEP
                                elif cmd_type == 'down':
                                    self.rc_values[2] = 1500 - CONTROL_STEP
                        
                        # 限制值在有效范围内
                        for i in range(len(self.rc_values)):
                            self.rc_values[i] = max(1000, min(self.rc_values[i], 2000))
                
                # 发送RC命令
                self.send_rc_command()
                
            except Exception as e:
                print(f"控制循环错误: {str(e)}")
            
            rate.sleep()

    def get_status(self):
        """获取控制器状态"""
        with self.command_lock:
            active_continuous = [cmd for cmd, active in self.continuous_commands.items() if active]
            return {
                'single_command_active': self.is_command_active,
                'current_single_command': self.current_command,
                'active_continuous_commands': active_continuous,
                'rc_values': self.rc_values.copy()
            }

# 全局控制器实例
manual_controller = None

def initialize_manual_controller():
    """初始化手动控制器"""
    global manual_controller
    if manual_controller is None:
        try:
            manual_controller = ManualRCController()
            print("手动控制器初始化成功")
            return True
        except Exception as e:
            print(f"手动控制器初始化失败: {str(e)}")
            return False
    return True

def get_manual_controller():
    """获取手动控制器实例"""
    global manual_controller
    if manual_controller is None:
        initialize_manual_controller()
    return manual_controller

# 便捷函数接口
def execute_command(command_type, duration=None):
    """执行单次命令"""
    controller = get_manual_controller()
    if controller:
        controller.execute_single_command(command_type, duration)

def start_continuous(command_type):
    """开始持续命令"""
    controller = get_manual_controller()
    if controller:
        controller.start_continuous_command(command_type)

def stop_continuous(command_type):
    """停止持续命令"""
    controller = get_manual_controller()
    if controller:
        controller.stop_continuous_command(command_type)

def stop_all():
    """停止所有命令"""
    controller = get_manual_controller()
    if controller:
        controller.stop_all_commands()

def get_controller_status():
    """获取控制器状态"""
    controller = get_manual_controller()
    if controller:
        return controller.get_status()
    return None

if __name__ == '__main__':
    """测试模式"""
    try:
        rospy.init_node('manual_controller_test', anonymous=True)
        
        # 初始化控制器
        if initialize_manual_controller():
            print("手动控制器测试模式启动")
            print("可用命令: forward, backward, left, right, up, down")
            
            # 测试单次命令
            time.sleep(2)
            execute_command('forward', 2.0)
            time.sleep(3)
            execute_command('right', 1.5)
            time.sleep(2)
            
            print("测试完成")
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        print("手动控制器测试中断")
    except KeyboardInterrupt:
        print("手动控制器测试停止")
