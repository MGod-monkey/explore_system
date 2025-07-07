# 改进的小球位置跟踪器，支持图像缓存和截图功能
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
from collections import deque
import threading
import time

class BallPoseTracker:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('ball_pose_tracker', anonymous=True)

        # 创建发布器，用于发布可视化标记
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

        # 订阅小球坐标的话题
        rospy.Subscriber('/ball_pose_tracking', PoseStamped, self.pose_callback)
        
        # 订阅图像话题（用于截图）
        rospy.Subscriber('/image_converter/output_video', Image, self.image_callback)

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # 用于存储接收到的小球坐标
        self.positions = []
        self.history_positions = []  # 存储历史位置
        self.next_id = 0  # 用于给每个小球分配唯一ID
        self.latest_pose_time = rospy.Time.now()
        
        # 图像缓存相关
        self.image_buffer = deque(maxlen=100)  # 缓存最近100帧图像
        self.current_detection_images = []  # 当前检测期间的图像
        self.is_detecting = False  # 是否正在检测小球
        self.detection_start_time = None
        
        # 位置差异阈值（米）
        self.position_threshold = 1.5
        
        # 截图保存目录
        self.screenshots_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ball_screenshots")
        if not os.path.exists(self.screenshots_dir):
            os.makedirs(self.screenshots_dir)

        # 发布频率
        self.rate = rospy.Rate(50)
        
        # 线程锁
        self.lock = threading.Lock()

    def image_callback(self, msg):
        """图像话题回调函数，缓存图像"""
        try:
            # 转换ROS图像为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            with self.lock:
                # 添加时间戳信息
                image_data = {
                    'image': cv_image.copy(),
                    'timestamp': msg.header.stamp,
                    'time_sec': rospy.Time.now().to_sec()
                }
                
                # 添加到循环缓冲区
                self.image_buffer.append(image_data)
                
                # 如果正在检测小球，也添加到检测期间的图像列表
                if self.is_detecting:
                    self.current_detection_images.append(image_data)
                    
        except Exception as e:
            rospy.logerr(f"图像处理错误: {str(e)}")

    def pose_callback(self, msg):
        """小球位置回调函数"""
        with self.lock:
            # 如果这是检测的开始
            if not self.is_detecting:
                self.is_detecting = True
                self.detection_start_time = rospy.Time.now()
                self.current_detection_images = []
                rospy.loginfo("开始检测小球，开始缓存图像")
            
            # 接收到坐标后存储 x, y, z 坐标
            self.positions.append([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            self.latest_pose_time = rospy.Time.now()

    def is_same_ball(self, new_position):
        """检查新位置是否与已有小球位置相近（认为是同一个小球）"""
        for pos_data in self.history_positions:
            existing_position = pos_data['position']
            # 检查x和y坐标是否都在阈值范围内
            if (abs(new_position[0] - existing_position[0]) < self.position_threshold and 
                abs(new_position[1] - existing_position[1]) < self.position_threshold):
                return True
        return False

    def select_best_image(self, images):
        """从检测期间的图像中选择最佳图像"""
        if not images:
            return None
            
        # 简单策略：选择中间时间点的图像
        # 你可以根据需要实现更复杂的选择策略，比如：
        # - 选择图像质量最好的（清晰度最高）
        # - 选择小球最居中的
        # - 选择亮度最合适的
        
        middle_index = len(images) // 2
        return images[middle_index]['image']

    def save_ball_screenshot(self, ball_id, image):
        """保存小球截图"""
        try:
            filename = f"ball_{ball_id}_{int(time.time())}.jpg"
            filepath = os.path.join(self.screenshots_dir, filename)
            cv2.imwrite(filepath, image)
            rospy.loginfo(f"保存小球 {ball_id} 截图: {filepath}")
            return filepath
        except Exception as e:
            rospy.logerr(f"保存截图失败: {str(e)}")
            return None

    def process_positions(self):
        """处理位置数据并生成截图"""
        if len(self.positions) < 3:
            return None, None
        
        positions_np = np.array(self.positions)
        mean = np.mean(positions_np, axis=0)
        std_dev = np.std(positions_np, axis=0)

        filtered_positions = [pos for pos in positions_np if np.all(np.abs(pos - mean) <= 2 * std_dev)]

        if len(filtered_positions) == 0:
            return None, None

        filtered_positions_np = np.array(filtered_positions)
        average_position = np.mean(filtered_positions_np, axis=0)

        # 检查新位置是否与已有小球位置相近
        if not self.is_same_ball(average_position):
            # 选择最佳图像
            best_image = self.select_best_image(self.current_detection_images)
            
            # 保存截图
            screenshot_path = None
            if best_image is not None:
                screenshot_path = self.save_ball_screenshot(self.next_id, best_image)
            
            # 添加到历史记录
            ball_data = {
                'position': average_position,
                'id': self.next_id,
                'screenshot_path': screenshot_path,
                'detection_time': rospy.Time.now(),
                'image_count': len(self.current_detection_images)
            }
            self.history_positions.append(ball_data)
            
            self.next_id += 1
            return average_position, screenshot_path
        else:
            # 如果是同一个小球，则不添加到历史记录
            return None, None

    def publish_markers(self):
        """发布所有小球的标记"""
        # 遍历并显示所有历史位置
        for pos_data in self.history_positions:
            # 发布小球标记
            ball_marker = Marker()
            ball_marker.header.frame_id = "world"
            ball_marker.header.stamp = rospy.Time.now()
            ball_marker.ns = "demo"
            ball_marker.id = pos_data['id'] * 2  # 每个小球占用两个ID(球体和文本)
            ball_marker.type = Marker.SPHERE
            ball_marker.action = Marker.ADD

            pos = pos_data['position']
            ball_marker.pose.position.x = pos[0]
            ball_marker.pose.position.y = pos[1]
            ball_marker.pose.position.z = pos[2]

            ball_marker.scale.x = 1.0
            ball_marker.scale.y = 1.0
            ball_marker.scale.z = 1.0

            ball_marker.color.r = 1.0
            ball_marker.color.g = 0.0
            ball_marker.color.b = 0.0
            ball_marker.color.a = 1.0

            self.marker_pub.publish(ball_marker)

            # 发布文本标记
            text_marker = Marker()
            text_marker.header.frame_id = "world"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "demo"
            text_marker.id = pos_data['id'] * 2 + 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = pos[0]
            text_marker.pose.position.y = pos[1]
            text_marker.pose.position.z = pos[2] + 1.5

            # 包含截图信息的文本
            screenshot_info = "有截图" if pos_data.get('screenshot_path') else "无截图"
            text_marker.text = "Ball {}\nx: {:.2f}m, y: {:.2f}m, z: {:.2f}m\n{}".format(
                pos_data['id'], pos[0], pos[1], pos[2], screenshot_info)
            
            text_marker.scale.z = 1.0
            text_marker.color.r = 0.0
            text_marker.color.g = 0.0
            text_marker.color.b = 0.0
            text_marker.color.a = 1.0

            self.marker_pub.publish(text_marker)

    def run(self):
        """主运行循环"""
        while not rospy.is_shutdown():
            with self.lock:
                # 如果超过1秒没有接收到新的坐标，则处理已有坐标
                if (rospy.Time.now() - self.latest_pose_time).to_sec() > 1.0 and len(self.positions) > 0:
                    rospy.loginfo(f"检测结束，处理 {len(self.positions)} 个位置点，缓存了 {len(self.current_detection_images)} 张图像")
                    
                    average_position, screenshot_path = self.process_positions()
                    if average_position is not None:
                        rospy.loginfo(f"新小球位置: {average_position}, 截图: {screenshot_path}")
                        self.publish_markers()  # 更新显示所有历史位置
                    
                    # 清空当前处理的数据
                    self.positions = []
                    self.current_detection_images = []
                    self.is_detecting = False
                    self.detection_start_time = None

            self.rate.sleep()

if __name__ == '__main__':
    try:
        tracker = BallPoseTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass
