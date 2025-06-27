#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import os
import rospy
from sensor_msgs.msg import BatteryState, Image, NavSatFix, Imu
from mavros_msgs.msg import State, RCIn
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32, Float64
from visualization_msgs.msg import Marker
import threading
import time
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError

class TopicsSubscriber:
    def __init__(self, config_file="topics_config.json"):
        # 初始化数据存储字典
        self.data = {
            "battery": {
                "voltage": 0.0,
                "current": 0.0, 
                "percentage": 0.0,
                "temperature": 0.0
            },
            "status": {
                "connected": False,
                "armed": False,
                "guided": False,
                "mode": ""
            },
            "odometry": {
                "position": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                },
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "w": 0.0
                }
            },
            "velocity": {
                "linear": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                },
                "angular": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                },
                "speed": 0.0  # 合成的线速度
            },
            "camera": {
                "image": None,  # 存储OpenCV格式的图像
                "width": 0,
                "height": 0,
                "encoding": ""
            },
            "depth": {
                "image": None,  # 存储OpenCV格式的深度图像
                "width": 0,
                "height": 0,
                "encoding": ""
            },
            "marker": {
                "id": 0,
                "type": 0,
                "pose": {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                },
                "scale": {"x": 0.0, "y": 0.0, "z": 0.0},
                "color": {"r": 0.0, "g": 0.0, "b": 0.0, "a": 0.0},
                "action": 0,
                "text": ""
            },
            "attitude": {
                "pitch": 0.0,       # 俯仰角
                "roll": 0.0,        # 滚转角
                "yaw": 0.0,         # 偏航角
                "timestamp": 0.0    # 时间戳
            },
            "mavros_state": {
                "connected": False,
                "armed": False,
                "guided": False,
                "mode": ""
            },
            "mavros_gps": {
                "status": 0,
                "latitude": 0.0,
                "longitude": 0.0,
                "altitude": 0.0
            },
            "mavros_battery": {
                "voltage": 0.0,
                "current": 0.0, 
                "percentage": 0.0,
                "temperature": 0.0
            },
            "mavros_altitude": {
                "relative": 0.0
            },
            "mavros_velocity": {
                "linear": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                },
                "angular": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                }
            },
            "rc_input": {
                "channels": [],
                "rssi": 0
            }
        }
        
        # 回调函数字典
        self.callbacks = {}
        
        # 订阅者字典，用于跟踪订阅状态
        self.subscribers = {}
        
        # 话题是否活跃状态
        self.topics_active = {
            "battery": False,
            "status": False,
            "odometry": False,
            "velocity": False,
            "camera": False,
            "depth": False,
            "bird_view": False,
            "marker": False,
            "attitude": False,
            "mavros_state": False,
            "mavros_gps": False,
            "mavros_battery": False,
            "mavros_altitude": False,
            "mavros_velocity": False,
            "rc_input": False
        }
        
        # 初始化cv_bridge
        self.bridge = CvBridge()
        
        # 加载配置文件
        self.load_config(config_file)
        
        # 初始化ROS订阅者(异步方式)
        self.running = True
        self.subscriber_thread = threading.Thread(target=self.monitor_topics)
        self.subscriber_thread.daemon = True
        self.subscriber_thread.start()
    
    def load_config(self, config_file):
        """加载话题订阅配置文件"""
        try:
            with open(config_file, 'r') as f:
                self.config = json.load(f)
            rospy.loginfo(f"成功加载配置文件: {config_file}")
        except Exception as e:
            rospy.logerr(f"加载配置文件失败: {str(e)}")
            self.config = {}
    
    def monitor_topics(self):
        """异步监控话题，当话题可用时自动订阅"""
        while self.running and not rospy.is_shutdown():
            try:
                # 检查并订阅可用的话题
                self.check_and_subscribe_topics()
                
                # 每秒检查一次
                time.sleep(1.0)
            except Exception as e:
                rospy.logerr(f"监控话题时出错: {str(e)}")
                time.sleep(2.0)
    
    def check_and_subscribe_topics(self):
        """检查并订阅可用的话题"""
        # 获取当前发布的所有话题
        published_topics = dict(rospy.get_published_topics())
        
        # 检查并订阅电池话题
        if "battery" in self.config and self.config["battery"]["topic"] in published_topics:
            if "battery" not in self.subscribers or not self.subscribers["battery"]:
                try:
                    self.subscribers["battery"] = rospy.Subscriber(
                        self.config["battery"]["topic"],
                        BatteryState,
                        self.battery_callback
                    )
                    self.topics_active["battery"] = True
                    rospy.loginfo(f"成功订阅电池话题: {self.config['battery']['topic']}")
                except Exception as e:
                    rospy.logerr(f"订阅电池话题失败: {str(e)}")
        
        # 检查并订阅状态话题
        if "status" in self.config and self.config["status"]["topic"] in published_topics:
            if "status" not in self.subscribers or not self.subscribers["status"]:
                try:
                    self.subscribers["status"] = rospy.Subscriber(
                        self.config["status"]["topic"],
                        State,
                        self.state_callback
                    )
                    self.topics_active["status"] = True
                    rospy.loginfo(f"成功订阅状态话题: {self.config['status']['topic']}")
                except Exception as e:
                    rospy.logerr(f"订阅状态话题失败: {str(e)}")
        
        # 检查并订阅位置话题
        if "odometry" in self.config and self.config["odometry"]["topic"] in published_topics:
            if "odometry" not in self.subscribers or not self.subscribers["odometry"]:
                try:
                    self.subscribers["odometry"] = rospy.Subscriber(
                        self.config["odometry"]["topic"],
                        Odometry,
                        self.odometry_callback
                    )
                    self.topics_active["odometry"] = True
                    rospy.loginfo(f"成功订阅位置话题: {self.config['odometry']['topic']}")
                except Exception as e:
                    rospy.logerr(f"订阅位置话题失败: {str(e)}")
        
        # 检查并订阅速度话题
        if "velocity" in self.config and self.config["velocity"]["topic"] in published_topics:
            if "velocity" not in self.subscribers or not self.subscribers["velocity"]:
                try:
                    self.subscribers["velocity"] = rospy.Subscriber(
                        self.config["velocity"]["topic"],
                        TwistStamped,
                        self.velocity_callback
                    )
                    self.topics_active["velocity"] = True
                    rospy.loginfo(f"成功订阅速度话题: {self.config['velocity']['topic']}")
                except Exception as e:
                    rospy.logerr(f"订阅速度话题失败: {str(e)}")
                    
        # 检查并订阅图像话题
        if "camera" in self.config and self.config["camera"]["topic"] in published_topics:
            if "camera" not in self.subscribers or not self.subscribers["camera"]:
                try:
                    self.subscribers["camera"] = rospy.Subscriber(
                        self.config["camera"]["topic"],
                        Image,
                        self.camera_callback
                    )
                    self.topics_active["camera"] = True
                    rospy.loginfo(f"成功订阅图像话题: {self.config['camera']['topic']}")
                except Exception as e:
                    rospy.logerr(f"订阅图像话题失败: {str(e)}")
                    
        # 检查并订阅深度图像话题
        if "depth" in self.config and self.config["depth"]["topic"] in published_topics:
            if "depth" not in self.subscribers or not self.subscribers["depth"]:
                try:
                    self.subscribers["depth"] = rospy.Subscriber(
                        self.config["depth"]["topic"],
                        Image,
                        self.depth_callback
                    )
                    self.topics_active["depth"] = True
                    rospy.loginfo(f"成功订阅深度图像话题: {self.config['depth']['topic']}")
                except Exception as e:
                    rospy.logerr(f"订阅深度图像话题失败: {str(e)}")
        
                            # 检查并订阅鸟瞰图话题
        if "bird_view" in self.config:
            # 尝试强制订阅鸟瞰图话题，不管是否在已发布话题列表中
            if "bird_view" not in self.subscribers or not self.subscribers["bird_view"]:
                try:
                    print(f"尝试订阅鸟瞰图话题: {self.config['bird_view']['topic']}")
                    self.subscribers["bird_view"] = rospy.Subscriber(
                        self.config["bird_view"]["topic"],
                        Image,
                        self.bird_view_callback,
                        queue_size=1
                    )
                    self.topics_active["bird_view"] = True
                    rospy.loginfo(f"成功订阅鸟瞰图话题: {self.config['bird_view']['topic']}")
                except Exception as e:
                    rospy.logerr(f"订阅鸟瞰图话题失败: {str(e)}")
                    
        # 检查并订阅MAVROS状态话题
        if "mavros_state" in self.config and self.config["mavros_state"]["topic"] in published_topics:
            if "mavros_state" not in self.subscribers or not self.subscribers["mavros_state"]:
                try:
                    self.subscribers["mavros_state"] = rospy.Subscriber(
                        self.config["mavros_state"]["topic"],
                        State,
                        self.mavros_state_callback
                    )
                    self.topics_active["mavros_state"] = True
                    rospy.loginfo(f"成功订阅MAVROS状态话题: {self.config['mavros_state']['topic']}")
                except Exception as e:
                    rospy.logerr(f"订阅MAVROS状态话题失败: {str(e)}")
                    
        # 检查并订阅MAVROS GPS话题
        if "mavros_gps" in self.config and self.config["mavros_gps"]["topic"] in published_topics:
            if "mavros_gps" not in self.subscribers or not self.subscribers["mavros_gps"]:
                try:
                    self.subscribers["mavros_gps"] = rospy.Subscriber(
                        self.config["mavros_gps"]["topic"],
                        NavSatFix,
                        self.mavros_gps_callback
                    )
                    self.topics_active["mavros_gps"] = True
                    rospy.loginfo(f"成功订阅MAVROS GPS话题: {self.config['mavros_gps']['topic']}")
                except Exception as e:
                    rospy.logerr(f"订阅MAVROS GPS话题失败: {str(e)}")
                    
        # 检查并订阅MAVROS电池话题
        if "mavros_battery" in self.config and self.config["mavros_battery"]["topic"] in published_topics:
            if "mavros_battery" not in self.subscribers or not self.subscribers["mavros_battery"]:
                try:
                    self.subscribers["mavros_battery"] = rospy.Subscriber(
                        self.config["mavros_battery"]["topic"],
                        BatteryState,
                        self.mavros_battery_callback
                    )
                    self.topics_active["mavros_battery"] = True
                    rospy.loginfo(f"成功订阅MAVROS电池话题: {self.config['mavros_battery']['topic']}")
                except Exception as e:
                    rospy.logerr(f"订阅MAVROS电池话题失败: {str(e)}")
                    
        # 检查并订阅MAVROS相对高度话题
        if "mavros_altitude" in self.config and self.config["mavros_altitude"]["topic"] in published_topics:
            if "mavros_altitude" not in self.subscribers or not self.subscribers["mavros_altitude"]:
                try:
                    self.subscribers["mavros_altitude"] = rospy.Subscriber(
                        self.config["mavros_altitude"]["topic"],
                        Float64,
                        self.mavros_altitude_callback
                    )
                    self.topics_active["mavros_altitude"] = True
                    rospy.loginfo(f"成功订阅MAVROS高度话题: {self.config['mavros_altitude']['topic']}")
                except Exception as e:
                    rospy.logerr(f"订阅MAVROS高度话题失败: {str(e)}")
                    
        # 检查并订阅MAVROS速度话题
        if "mavros_velocity" in self.config and self.config["mavros_velocity"]["topic"] in published_topics:
            if "mavros_velocity" not in self.subscribers or not self.subscribers["mavros_velocity"]:
                try:
                    self.subscribers["mavros_velocity"] = rospy.Subscriber(
                        self.config["mavros_velocity"]["topic"],
                        TwistStamped,
                        self.mavros_velocity_callback
                    )
                    self.topics_active["mavros_velocity"] = True
                    rospy.loginfo(f"成功订阅MAVROS速度话题: {self.config['mavros_velocity']['topic']}")
                except Exception as e:
                    rospy.logerr(f"订阅MAVROS速度话题失败: {str(e)}")
                    
        # 检查并订阅RC输入话题
        if "rc_input" in self.config and self.config["rc_input"]["topic"] in published_topics:
            if "rc_input" not in self.subscribers or not self.subscribers["rc_input"]:
                try:
                    self.subscribers["rc_input"] = rospy.Subscriber(
                        self.config["rc_input"]["topic"],
                        RCIn,
                        self.rc_input_callback
                    )
                    self.topics_active["rc_input"] = True
                    rospy.loginfo(f"成功订阅RC输入话题: {self.config['rc_input']['topic']}")
                except Exception as e:
                    rospy.logerr(f"订阅RC输入话题失败: {str(e)}")
                    print(f"订阅鸟瞰图话题时出错: {str(e)}")
                    self.topics_active["bird_view"] = False
        
        # 检查并订阅标记点话题
        if "marker" in self.config and self.config["marker"]["topic"] in published_topics:
            if "marker" not in self.subscribers or not self.subscribers["marker"]:
                try:
                    self.subscribers["marker"] = rospy.Subscriber(
                        self.config["marker"]["topic"],
                        Marker,
                        self.marker_callback
                    )
                    self.topics_active["marker"] = True
                    rospy.loginfo(f"成功订阅标记点话题: {self.config['marker']['topic']}")
                except Exception as e:
                    rospy.logerr(f"订阅标记点话题失败: {str(e)}")
                    
        # 检查并订阅姿态数据话题
        if "attitude" in self.config and self.config["attitude"]["topic"] in published_topics:
            if "attitude" not in self.subscribers or not self.subscribers["attitude"]:
                try:
                    from sensor_msgs.msg import Imu
                    self.subscribers["attitude"] = rospy.Subscriber(
                        self.config["attitude"]["topic"],
                        Imu,
                        self.attitude_callback
                    )
                    self.topics_active["attitude"] = True
                    rospy.loginfo(f"成功订阅姿态数据话题: {self.config['attitude']['topic']}")
                except Exception as e:
                    rospy.logerr(f"订阅姿态数据话题失败: {str(e)}")
    
    def battery_callback(self, msg):
        """电池状态话题回调函数"""
        try:
            self.data["battery"]["voltage"] = msg.voltage
            self.data["battery"]["current"] = msg.current
            self.data["battery"]["percentage"] = msg.percentage
            self.data["battery"]["temperature"] = msg.temperature
            
            # 触发注册的回调函数
            if "battery" in self.callbacks:
                for callback in self.callbacks["battery"]:
                    try:
                        callback(self.data["battery"])
                    except Exception as e:
                        rospy.logerr(f"执行电池回调函数时出错: {str(e)}")
        except Exception as e:
            rospy.logerr(f"处理电池数据时出错: {str(e)}")
    
    def state_callback(self, msg):
        """无人机状态话题回调函数"""
        try:
            self.data["status"]["connected"] = msg.connected
            self.data["status"]["armed"] = msg.armed
            self.data["status"]["guided"] = msg.guided
            self.data["status"]["mode"] = msg.mode
            
            # 触发注册的回调函数
            if "status" in self.callbacks:
                for callback in self.callbacks["status"]:
                    try:
                        callback(self.data["status"])
                    except Exception as e:
                        rospy.logerr(f"执行状态回调函数时出错: {str(e)}")
        except Exception as e:
            rospy.logerr(f"处理状态数据时出错: {str(e)}")
                
    def odometry_callback(self, msg):
        """无人机位置话题回调函数"""
        try:
            # 更新位置信息
            self.data["odometry"]["position"]["x"] = msg.pose.pose.position.x
            self.data["odometry"]["position"]["y"] = msg.pose.pose.position.y
            self.data["odometry"]["position"]["z"] = msg.pose.pose.position.z
            
            # 更新方向信息
            self.data["odometry"]["orientation"]["x"] = msg.pose.pose.orientation.x
            self.data["odometry"]["orientation"]["y"] = msg.pose.pose.orientation.y
            self.data["odometry"]["orientation"]["z"] = msg.pose.pose.orientation.z
            self.data["odometry"]["orientation"]["w"] = msg.pose.pose.orientation.w
            
            # 如果速度话题未激活，也可以从里程计消息中提取速度
            if not self.topics_active["velocity"]:
                # 提取线速度
                self.data["velocity"]["linear"]["x"] = msg.twist.twist.linear.x
                self.data["velocity"]["linear"]["y"] = msg.twist.twist.linear.y
                self.data["velocity"]["linear"]["z"] = msg.twist.twist.linear.z
                
                # 提取角速度
                self.data["velocity"]["angular"]["x"] = msg.twist.twist.angular.x
                self.data["velocity"]["angular"]["y"] = msg.twist.twist.angular.y
                self.data["velocity"]["angular"]["z"] = msg.twist.twist.angular.z
                
                # 计算合成速度(cm/s)
                linear_x = self.data["velocity"]["linear"]["x"]
                linear_y = self.data["velocity"]["linear"]["y"]
                linear_z = self.data["velocity"]["linear"]["z"]
                speed = math.sqrt(linear_x**2 + linear_y**2 + linear_z**2) * 100  # 转换为厘米/秒
                self.data["velocity"]["speed"] = speed
                
                # 触发速度回调
                if "velocity" in self.callbacks:
                    for callback in self.callbacks["velocity"]:
                        try:
                            callback(self.data["velocity"])
                        except Exception as e:
                            rospy.logerr(f"执行速度回调函数时出错: {str(e)}")
            
            # 触发注册的回调函数
            if "odometry" in self.callbacks:
                for callback in self.callbacks["odometry"]:
                    try:
                        callback(self.data["odometry"])
                    except Exception as e:
                        rospy.logerr(f"执行位置回调函数时出错: {str(e)}")
        except Exception as e:
            rospy.logerr(f"处理位置数据时出错: {str(e)}")
    
    def velocity_callback(self, msg):
        """无人机速度话题回调函数"""
        try:
            # 更新线速度
            self.data["velocity"]["linear"]["x"] = msg.twist.linear.x
            self.data["velocity"]["linear"]["y"] = msg.twist.linear.y
            self.data["velocity"]["linear"]["z"] = msg.twist.linear.z
            
            # 更新角速度
            self.data["velocity"]["angular"]["x"] = msg.twist.angular.x
            self.data["velocity"]["angular"]["y"] = msg.twist.angular.y
            self.data["velocity"]["angular"]["z"] = msg.twist.angular.z
            
            # 计算合成速度(cm/s)
            linear_x = self.data["velocity"]["linear"]["x"]
            linear_y = self.data["velocity"]["linear"]["y"]
            linear_z = self.data["velocity"]["linear"]["z"]
            speed = math.sqrt(linear_x**2 + linear_y**2 + linear_z**2) * 100  # 转换为厘米/秒
            self.data["velocity"]["speed"] = speed
            
            # 触发注册的回调函数
            if "velocity" in self.callbacks:
                for callback in self.callbacks["velocity"]:
                    try:
                        callback(self.data["velocity"])
                    except Exception as e:
                        rospy.logerr(f"执行速度回调函数时出错: {str(e)}")
        except Exception as e:
            rospy.logerr(f"处理速度数据时出错: {str(e)}")
    
    def camera_callback(self, msg):
        """摄像头图像话题回调函数"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 更新图像数据
            self.data["camera"]["image"] = cv_image
            self.data["camera"]["width"] = msg.width
            self.data["camera"]["height"] = msg.height
            self.data["camera"]["encoding"] = msg.encoding
            
            # 触发注册的回调函数
            if "camera" in self.callbacks:
                for callback in self.callbacks["camera"]:
                    try:
                        callback(self.data["camera"])
                    except Exception as e:
                        rospy.logerr(f"执行图像回调函数时出错: {str(e)}")
        except CvBridgeError as e:
            rospy.logerr(f"转换图像数据时出错: {str(e)}")
        except Exception as e:
            rospy.logerr(f"处理图像数据时出错: {str(e)}")
    
    def depth_callback(self, msg):
        """深度图像话题回调函数"""
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            
            # 存储图像和相关信息
            self.data["depth"]["image"] = cv_image
            self.data["depth"]["width"] = msg.width
            self.data["depth"]["height"] = msg.height
            self.data["depth"]["encoding"] = msg.encoding
            
            # 触发注册的回调函数
            if "depth" in self.callbacks:
                for callback in self.callbacks["depth"]:
                    callback(self.data["depth"])
                    
        except CvBridgeError as e:
            rospy.logerr(f"转换深度图像失败: {str(e)}")
        except Exception as e:
            rospy.logerr(f"处理深度图像时出错: {str(e)}")
            
    def bird_view_callback(self, msg):
        """鸟瞰图话题回调函数"""
        try:            
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            
            # 检查图像是否有效
            if cv_image is None or cv_image.size == 0:
                return
            
            # 如果self.data中还没有bird_view字段，先创建它
            if "bird_view" not in self.data:
                self.data["bird_view"] = {
                    "image": None,
                    "width": 0,
                    "height": 0,
                    "encoding": ""
                }
            
            # 存储图像和相关信息
            self.data["bird_view"]["image"] = cv_image
            self.data["bird_view"]["width"] = msg.width
            self.data["bird_view"]["height"] = msg.height
            self.data["bird_view"]["encoding"] = msg.encoding
            
            # 触发注册的回调函数
            if "bird_view" in self.callbacks:
                for callback in self.callbacks["bird_view"]:
                    callback(self.data["bird_view"])
                    
        except CvBridgeError as e:
            print(f"转换鸟瞰图失败: {str(e)}")
        except Exception as e:
            print(f"处理鸟瞰图话题时出错: {str(e)}")
    
    def marker_callback(self, msg):
        """标记点话题回调函数"""
        try:
            # 只处理小球标记(SPHERE类型)
            if msg.type == Marker.SPHERE:
                # 更新标记点数据
                self.data["marker"]["id"] = msg.id
                self.data["marker"]["type"] = msg.type
                
                # 更新位置信息
                self.data["marker"]["pose"]["position"]["x"] = msg.pose.position.x
                self.data["marker"]["pose"]["position"]["y"] = msg.pose.position.y
                self.data["marker"]["pose"]["position"]["z"] = msg.pose.position.z
                
                # 更新方向信息
                self.data["marker"]["pose"]["orientation"]["x"] = msg.pose.orientation.x
                self.data["marker"]["pose"]["orientation"]["y"] = msg.pose.orientation.y
                self.data["marker"]["pose"]["orientation"]["z"] = msg.pose.orientation.z
                self.data["marker"]["pose"]["orientation"]["w"] = msg.pose.orientation.w
                
                # 更新尺寸信息
                self.data["marker"]["scale"]["x"] = msg.scale.x
                self.data["marker"]["scale"]["y"] = msg.scale.y
                self.data["marker"]["scale"]["z"] = msg.scale.z
                
                # 更新颜色信息
                self.data["marker"]["color"]["r"] = msg.color.r
                self.data["marker"]["color"]["g"] = msg.color.g
                self.data["marker"]["color"]["b"] = msg.color.b
                self.data["marker"]["color"]["a"] = msg.color.a
                
                # 更新动作信息
                self.data["marker"]["action"] = msg.action
                
                # 更新文本信息(如果有)
                if hasattr(msg, "text"):
                    self.data["marker"]["text"] = msg.text
                
                # 触发注册的回调函数
                if "marker" in self.callbacks:
                    for callback in self.callbacks["marker"]:
                        try:
                            callback(self.data["marker"])
                        except Exception as e:
                            rospy.logerr(f"执行标记点回调函数时出错: {str(e)}")
        except Exception as e:
            rospy.logerr(f"处理标记点数据时出错: {str(e)}")
    
    def attitude_callback(self, msg):
        """处理姿态数据，从IMU四元数转换为欧拉角"""
        try:
            # 从四元数转换为欧拉角
            orientation = msg.orientation
            quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
            
            # 使用tf转换库计算欧拉角
            import tf
            euler = tf.transformations.euler_from_quaternion(quaternion)
            
            # 欧拉角是按照roll, pitch, yaw的顺序
            roll = math.degrees(euler[0])
            pitch = math.degrees(euler[1])
            yaw = math.degrees(euler[2])
            
            # 更新姿态数据
            self.data["attitude"]["roll"] = roll
            self.data["attitude"]["pitch"] = pitch
            self.data["attitude"]["yaw"] = yaw
            self.data["attitude"]["timestamp"] = rospy.Time.now().to_sec()
            
            # 调用注册的回调函数
            if "attitude" in self.callbacks:
                self.callbacks["attitude"](self.data["attitude"])
                
        except Exception as e:
            rospy.logerr(f"处理姿态数据时出错: {str(e)}")
    
    def register_callback(self, topic_name, callback):
        """注册回调函数，当话题数据更新时触发"""
        if topic_name not in self.callbacks:
            self.callbacks[topic_name] = []
        self.callbacks[topic_name].append(callback)
    
    def get_data(self, topic_name):
        """获取指定话题的最新数据"""
        if topic_name in self.data:
            return self.data[topic_name]
        return None
        
    def get_latest_data(self, topic_name):
        """获取指定话题的最新数据，与get_data相同，为了API统一性"""
        return self.get_data(topic_name)
    
    def is_topic_active(self, topic_name):
        """检查指定话题是否活跃"""
        if topic_name in self.topics_active:
            return self.topics_active[topic_name]
        return False
    
    def shutdown(self):
        """关闭订阅者线程"""
        self.running = False
        if self.subscriber_thread.is_alive():
            self.subscriber_thread.join(1.0)
        
        # 取消所有订阅
        for topic_name, subscriber in self.subscribers.items():
            if subscriber:
                subscriber.unregister()
                
    def mavros_state_callback(self, msg):
        """处理MAVROS状态数据"""
        try:
            # 更新状态数据
            self.data["mavros_state"]["connected"] = msg.connected
            self.data["mavros_state"]["armed"] = msg.armed
            self.data["mavros_state"]["guided"] = msg.guided
            self.data["mavros_state"]["mode"] = msg.mode
            
            # 设置话题活跃状态
            self.topics_active["mavros_state"] = True
            
            # 触发注册的回调函数
            if "mavros_state" in self.callbacks:
                for callback in self.callbacks["mavros_state"]:
                    try:
                        callback(self.data["mavros_state"])
                    except Exception as e:
                        rospy.logerr(f"执行MAVROS状态回调函数时出错: {str(e)}")
        except Exception as e:
            rospy.logerr(f"处理MAVROS状态数据时出错: {str(e)}")
            
    def mavros_gps_callback(self, msg):
        """处理MAVROS GPS数据"""
        try:
            # 更新GPS数据
            self.data["mavros_gps"]["status"] = msg.status.status
            self.data["mavros_gps"]["latitude"] = msg.latitude
            self.data["mavros_gps"]["longitude"] = msg.longitude
            self.data["mavros_gps"]["altitude"] = msg.altitude
            
            # 设置话题活跃状态
            self.topics_active["mavros_gps"] = True
            
            # 触发注册的回调函数
            if "mavros_gps" in self.callbacks:
                for callback in self.callbacks["mavros_gps"]:
                    try:
                        callback(self.data["mavros_gps"])
                    except Exception as e:
                        rospy.logerr(f"执行MAVROS GPS回调函数时出错: {str(e)}")
        except Exception as e:
            rospy.logerr(f"处理MAVROS GPS数据时出错: {str(e)}")
    
    def mavros_battery_callback(self, msg):
        """处理MAVROS电池数据"""
        try:
            # 更新电池数据
            self.data["mavros_battery"]["voltage"] = msg.voltage
            self.data["mavros_battery"]["current"] = msg.current
            self.data["mavros_battery"]["percentage"] = msg.percentage
            self.data["mavros_battery"]["temperature"] = msg.temperature
            
            # 设置话题活跃状态
            self.topics_active["mavros_battery"] = True
            
            # 触发注册的回调函数
            if "mavros_battery" in self.callbacks:
                for callback in self.callbacks["mavros_battery"]:
                    try:
                        callback(self.data["mavros_battery"])
                    except Exception as e:
                        rospy.logerr(f"执行MAVROS电池回调函数时出错: {str(e)}")
        except Exception as e:
            rospy.logerr(f"处理MAVROS电池数据时出错: {str(e)}")
    
    def mavros_altitude_callback(self, msg):
        """处理MAVROS高度数据"""
        try:
            # 更新高度数据 (相对高度)
            self.data["mavros_altitude"]["relative"] = msg.data
            
            # 设置话题活跃状态
            self.topics_active["mavros_altitude"] = True
            
            # 触发注册的回调函数
            if "mavros_altitude" in self.callbacks:
                for callback in self.callbacks["mavros_altitude"]:
                    try:
                        callback(self.data["mavros_altitude"])
                    except Exception as e:
                        rospy.logerr(f"执行MAVROS高度回调函数时出错: {str(e)}")
        except Exception as e:
            rospy.logerr(f"处理MAVROS高度数据时出错: {str(e)}")
    
    def mavros_velocity_callback(self, msg):
        """处理MAVROS速度数据"""
        try:
            # 更新线速度数据
            self.data["mavros_velocity"]["linear"]["x"] = msg.twist.linear.x
            self.data["mavros_velocity"]["linear"]["y"] = msg.twist.linear.y
            self.data["mavros_velocity"]["linear"]["z"] = msg.twist.linear.z
            
            # 更新角速度数据
            self.data["mavros_velocity"]["angular"]["x"] = msg.twist.angular.x
            self.data["mavros_velocity"]["angular"]["y"] = msg.twist.angular.y
            self.data["mavros_velocity"]["angular"]["z"] = msg.twist.angular.z
            
            # 设置话题活跃状态
            self.topics_active["mavros_velocity"] = True
            
            # 触发注册的回调函数
            if "mavros_velocity" in self.callbacks:
                for callback in self.callbacks["mavros_velocity"]:
                    try:
                        callback(self.data["mavros_velocity"])
                    except Exception as e:
                        rospy.logerr(f"执行MAVROS速度回调函数时出错: {str(e)}")
        except Exception as e:
            rospy.logerr(f"处理MAVROS速度数据时出错: {str(e)}")
    
    def rc_input_callback(self, msg):
        """处理RC遥控器输入数据"""
        try:
            # 更新RC数据
            self.data["rc_input"]["channels"] = list(msg.channels)
            self.data["rc_input"]["rssi"] = msg.rssi
            
            # 设置话题活跃状态
            self.topics_active["rc_input"] = True
            
            # 触发注册的回调函数
            if "rc_input" in self.callbacks:
                for callback in self.callbacks["rc_input"]:
                    try:
                        callback(self.data["rc_input"])
                    except Exception as e:
                        rospy.logerr(f"执行RC输入回调函数时出错: {str(e)}")
        except Exception as e:
            rospy.logerr(f"处理RC输入数据时出错: {str(e)}")