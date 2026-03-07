# 🚁 explore_system

一个基于 **ROS Noetic + PyQt5 + RViz** 的无人机导航可视化与控制系统。  
适用于仿真/实机调试场景，支持一键启动流程、实时状态监控、图像显示、动态障碍物跟踪与话题日志记录。

---

## ✨ 功能概览

- **一键流程控制**：按配置顺序启动/停止多进程（飞控、预测器、规划器等）
- **可视化控制台**：集成 RViz + 仪表面板 + 状态信息
- **多路图像显示**：RGB、深度、鸟瞰图实时刷新
- **障碍物监测**：展示障碍物距离、速度、加速度等信息
- **飞行任务操作**：目标点设置、一键返航、运行状态监控
- **话题日志系统**：关键 ROS Topic 订阅与日志落盘

---

## 🧱 技术栈

- Ubuntu 20.04
- ROS Noetic
- Python 3.8+
- PyQt5
- OpenCV / NumPy / psutil

---

## 📁 项目结构

```text
explore_system/
├── start.py                 # 主入口（启动 UI + 系统流程）
├── dashboard.py             # 控制中心 UI
├── manual_controller.py     # 手动控制模块
├── waypoint_dialog.py       # 目标点设置对话框
├── topics_subscriber.py     # ROS 话题订阅器
├── topic_logger.py          # 话题日志记录
├── utils.py                 # 通用工具与常量
├── processes_config.json    # 进程启动/停止配置
├── topics_config.json       # 话题映射配置
├── my_config.rviz           # RViz 配置
├── quick_install.sh         # 一键安装脚本（依赖 + 打包 + 桌面图标）
└── resource/                # UI 资源文件
```

---

## 🚀 快速开始

### 1) 环境准备

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws_dyn/devel/setup.bash
```

### 2) 安装 Python 依赖

```bash
pip3 install numpy opencv-python psutil PyQt5
```

### 3) 启动程序

```bash
cd ~/explore_system
python3 start.py
```

> 如果你希望一键安装与打包，可使用：
>
> ```bash
> bash quick_install.sh
> ```

---

## ⚙️ 关键配置

### 1. `topics_config.json`（必须）
将话题名改成你自己工程中的真实 Topic（例如电池、状态、里程计、图像、障碍物状态等）。

重点检查：
- `battery.topic`
- `status.topic`
- `odometry.topic`
- `camera.topic`
- `depth.topic`
- `bird_view.topic`
- `obstacle_states.topic`

### 2. `processes_config.json`（必须）
定义一键启动/停止的进程列表和顺序。

重点检查：
- `catkin_workspace`：你的工作空间路径
- `processes[].start_command`：每个节点的启动命令
- `processes[].order`：启动顺序
- `processes[].wait_seconds`：启动间隔

### 3. `my_config.rviz`（建议）
使用你项目对应的 RViz 配置覆盖该文件，确保显示项和坐标系匹配。

### 4. 目标点发布话题（按需）
在 `waypoint_dialog.py` 中确认目标点话题，例如：

```python
self.goal_publisher = rospy.Publisher(
    '/move_base_simple/goal',
    PoseStamped,
    queue_size=10
)
```

---

## 🎮 操作说明

- **一键启动**：按配置顺序启动所有核心模块
- **前往目标**：打开目标点设置并发布目标
- **一键返航**：回到原点并执行降落流程
- **停止程序**：终止配置中的运行进程

---

## 🐛 常见问题

1. **界面无数据/话题不更新**  
   - 检查 `topics_config.json` 是否与当前系统话题一致
   - 使用 `rostopic list` / `rostopic echo` 验证数据源

2. **RViz 显示异常或无模型**  
   - 检查 `my_config.rviz` 的 Fixed Frame 是否正确
   - 确认 TF 树正常发布

3. **障碍物列表不刷新**  
   - 确认 `obj_state_msgs` 已正确编译并 source
   - 确认 `/onboard_detector/states` 有持续输出

4. **一键启动后部分节点失败**  
   - 检查 `processes_config.json` 命令是否可单独运行
   - 适当增大 `wait_seconds`

---

## 📝 开发建议

- 将不同平台/场景（仿真、实机）拆分成多套 `processes_config.json`
- 为 `topics_config.json` 增加注释模板，便于迁移到新项目
- 建议加入启动前自检（Topic 存在性、依赖包检查）

---

## 📄 License

MIT
