# 🚁 动态避障可视化界面（explore_system / `dyn_sim`）

这是一个面向 **无人机动态避障调试** 的可视化控制界面（ROS Noetic + PyQt5 + RViz）。
它不是“搜索系统”，而是聚焦在 **动态障碍物感知 + 规划状态观测 + 控制联调**。

---

## ✅ 这个分支的定位（很重要）

当前文档对应 **`dyn_sim` 分支**。该分支相对 `main` 做了明显调整：

- 移除了搜索系统相关脚本与打包链路（如 `ball_pose_tracker*`、`build_executable.py` 等）
- 新增/强化了动态避障可视化能力：
  - `waypoint_dialog.py`（多目标点导航）
  - `processes_config.json`（启动流程配置化）
  - `utils.py`（路径与配置加载统一）
  - `topics_subscriber.py`（更多 topic 数据与状态）
- UI 与流程围绕“仿真/联调”展开，不再是早期“搜索任务”流程

---

## ✨ 核心能力

- **一键启动/停止流程**（按配置顺序）
- **RViz + 面板联动可视化**
- **实时图像流显示**：RGB / 深度 / 鸟瞰图
- **动态障碍物状态表**：位置、速度、加速度、尺寸等
- **FSM 状态观察**：配合返航与到点判定
- **手动 RC 控制接口**（`/mavros/rc/override`）
- **话题日志窗口**（独立多窗口）

---

## 🧱 运行环境

- Ubuntu 20.04
- ROS Noetic
- Python 3.8+
- PyQt5
- OpenCV / NumPy / psutil

安装依赖：

```bash
pip3 install numpy opencv-python psutil PyQt5
```

---

## 🚀 快速启动

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws_dyn/devel/setup.bash

cd ~/explore_system
python3 start.py
```

---

## 📁 关键文件说明

```text
explore_system/
├── start.py               # 主窗口与系统流程（UI + 启停 + 回调）
├── dashboard.py           # 仪表盘/姿态等 UI 组件
├── topics_subscriber.py   # topic 异步订阅与数据聚合
├── topic_logger.py        # 话题日志窗口
├── manual_controller.py   # 手动RC控制（/mavros/rc/override）
├── waypoint_dialog.py     # 多目标点导航对话框
├── processes_config.json  # 启停流程配置（启动命令/顺序/等待）
├── topics_config.json     # 话题配置（按你的系统改）
├── my_config.rviz         # RViz 显示配置
└── utils.py               # 路径/配置加载/默认配置
```

---

## ⚙️ 你需要先改的配置

### 1) `topics_config.json`（必须）
按你自己的系统改 topic 名，重点关注：

- `/mavros/battery`
- `/mavros/state`
- `/vins_fusion/imu_propagate`
- `/mavros/local_position/velocity_local`
- `/camera/color/image_raw`
- `/camera/depth_aligned_to_color_and_infra1/image_raw`
- `/onboard_detector/bird_view`
- `/onboard_detector/states`
- `/drone_0_ego_planner_node/planning/fsm_state`

### 2) `processes_config.json`（必须）
一键启动依赖这个文件，重点改：

- `catkin_workspace`
- `processes[].start_command`
- `processes[].order`
- `processes[].wait_seconds`
- `save_log` / `log_directory`

### 3) `my_config.rviz`（建议）
替换成你工程对应的 RViz 配置，确保 Fixed Frame 与 TF 链正确。

---

## 🎮 主要操作

- **一键启动**：按配置顺序拉起核心节点
- **前往目标**：打开目标点对话框，支持多点导航
- **一键返航**：发布返航点 `(0, 0, 0.8)`，并基于 FSM 状态触发后续降落流程
- **日志显示**：打开 ROS 话题监控窗口
- **手动控制**：通过按钮触发 RC 覆写通道控制

---

## 🐛 常见问题

1. **界面有框但没数据**
   - 先 `rostopic list` 看话题是否存在
   - 再确认 `topics_config.json` 是否和你当前系统一致

2. **鸟瞰图一直空白**
   - 检查 `/onboard_detector/bird_view` 是否持续发布 `sensor_msgs/Image`

3. **障碍物表不更新**
   - 确认 `obj_state_msgs/ObjectsStates` 已正确编译并 source
   - 检查 `/onboard_detector/states` 是否有输出

4. **一键启动部分进程失败**
   - 单独执行 `start_command` 验证
   - 适当加大 `wait_seconds`

---

## 📄 License

MIT
