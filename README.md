# 无人机自主导航系统

一个基于 ROS 的无人机自主导航可视化系统，集成 RViz 3D 显示、实时数据监控和动态障碍物跟踪功能。

## ✨ 主要功能

- 🚁 **导航控制**: 一键启动导航系统、设置目标点、一键返航
- 📷 **实时图像**: RGB 图像、深度图像、鸟瞰图显示
- 🎯 **障碍物跟踪**: 实时显示动态障碍物距离、速度、加速度
- 📊 **数据可视化**: RViz 3D 可视化和飞行数据仪表盘
- **话题监控**: ROS 话题实时日志记录

## 🚀 启动方式

```bash
# 确保ROS环境已配置
source /opt/ros/noetic/setup.bash
source ~/catkin_ws_dyn/devel/setup.bash

# 启动程序
cd ~/explore_system
python3 start.py
```

## 📁 项目结构

```
explore_system/
├── start.py                    # 主程序入口
├── utils.py                   # 工具函数和常量 (新)
├── processes_config.json      # 进程配置文件 (新)
├── dashboard.py               # 控制中心UI组件
├── topics_subscriber.py       # ROS话题订阅器
├── topic_logger.py           # 话题日志记录器
├── waypoint_dialog.py        # 目标点设置对话框
├── topics_config.json        # 话题配置文件 ⚠️ 需要修改
├── my_config.rviz           # RViz配置文件 ⚠️ 需要修改
└── README.md
```

## 🔧 集成到您的工程

### 1. 修改话题配置 (`topics_config.json`)

根据您的工程修改以下话题：

```json
{
  "battery": {
    "topic": "/mavros/battery", // 电池状态话题
    "msg_type": "sensor_msgs/BatteryState"
  },
  "status": {
    "topic": "/mavros/state", // 飞控状态话题
    "msg_type": "mavros_msgs/State"
  },
  "odometry": {
    "topic": "/vins_fusion/imu_propagate", // 里程计话题
    "msg_type": "nav_msgs/Odometry"
  },
  "velocity": {
    "topic": "/mavros/local_position/velocity_local", // 速度话题
    "msg_type": "geometry_msgs/TwistStamped"
  },
  "camera": {
    "topic": "/camera/color/image_raw", // RGB相机话题
    "msg_type": "sensor_msgs/Image"
  },
  "depth": {
    "topic": "/camera/depth_aligned_to_color_and_infra1/image_raw", // 深度相机话题
    "msg_type": "sensor_msgs/Image"
  },
  "attitude": {
    "topic": "/mavros/imu/data", // IMU姿态话题
    "msg_type": "sensor_msgs/Imu"
  },
  "fsm_state": {
    "topic": "/drone_0_ego_planner_node/planning/fsm_state", // 规划器FSM状态
    "msg_type": "std_msgs/Int32"
  },
  "obstacle_states": {
    "topic": "/onboard_detector/states", // 动态障碍物状态
    "msg_type": "obj_state_msgs/ObjectsStates"
  }
}
```

### 2. 修改 RViz 配置 (`my_config.rviz`)

将您项目的 RViz 配置文件保存（例如 `your_project.rviz`），并用它替换本项目中的 `my_config.rviz` 文件。

### 3. 修改进程配置 (`processes_config.json`)

系统启动和停止的进程现在通过配置文件进行管理。请修改 `processes_config.json`：

```json
{
  "catkin_workspace": "~/catkin_ws_dyn", // 工作空间路径
  "log_directory": "log", // 日志保存目录
  "processes": [
    {
      "name": "px4ctrl",
      "display_name": "PX4 飞控控制器",
      "start_command": "roslaunch px4ctrl run_node.launch", // 启动命令
      "wait_seconds": 5, // 启动后等待时间
      "order": 1 // 启动顺序
    }
    // ... 其他进程
  ]
}
```

### 4. 修改目标点发布话题

在 `waypoint_dialog.py` 中修改目标点发布话题：

```python
self.goal_publisher = rospy.Publisher(
    '/move_base_simple/goal',  # 修改为您的目标点话题
    PoseStamped,
    queue_size=10
)
```

## � 依赖要求

- Ubuntu 20.04 LTS
- ROS Noetic
- Python 3.8+
- PyQt5
- opencv-python
- numpy

```bash
# 安装Python依赖
pip3 install numpy opencv-python psutil PyQt5
```

## 🎮 操作说明

| 按钮     | 功能                        |
| -------- | --------------------------- |
| 一键启动 | 启动导航系统所有节点        |
| 前往目标 | 打开目标点设置对话框        |
| 一键返航 | 返回原点(0,0,0.8)并自动降落 |
| 停止程序 | 停止所有导航节点            |
| 导入点云 | (待实现)                    |

## 🐛 故障排除

1. **话题不显示**: 检查 `topics_config.json` 中的话题名称是否正确
2. **RViz 显示异常**: 检查 `my_config.rviz` 中的 Fixed Frame 设置
3. **障碍物列表不更新**: 确保 `obj_state_msgs` 消息包已编译

## 📄 许可证

MIT License
