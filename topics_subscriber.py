#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import os
import rospy
from sensor_msgs.msg import BatteryState, Image
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32
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
            "camera": False
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