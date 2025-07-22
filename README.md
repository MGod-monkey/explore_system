# 无人机自主搜索系统

一个基于ROS的无人机自主搜索和救援系统，具有实时图像处理、目标跟踪和可视化功能。

## ✨ 主要功能

- 🚁 **无人机控制**: 完整的无人机飞行控制系统
- 📷 **实时图像处理**: RGB和深度图像实时显示和处理
- 🎯 **目标跟踪**: 智能目标检测和位置跟踪
- 📊 **数据可视化**: RViz 3D可视化和实时数据显示
- 💾 **截图功能**: 自动截图和图像管理
- 📝 **日志记录**: 完整的系统日志和数据记录

## 🚀 快速开始

### 一键安装
```bash
# 克隆项目（如果需要）
git clone <repository-url>
cd explore_system

# 运行一键安装脚本
./quick_install.sh
```

### 手动安装
详细的手动安装步骤请参考 [打包安装说明.md](./打包安装说明.md)

## 📋 系统要求

- **操作系统**: Ubuntu 20.04 LTS
- **ROS版本**: ROS Noetic
- **Python**: 3.8+
- **内存**: 4GB+ RAM
- **存储**: 2GB+ 可用空间

## 🎮 使用方法

### 启动程序
```bash
# 方法1: 通过桌面图标启动
# 在应用程序菜单中搜索"无人机自主搜索系统"

# 方法2: 命令行启动
~/drone_search_system/run_drone_system.sh

# 方法3: 直接运行
~/drone_search_system/drone_search_system
```

### 基本操作
1. **启动系统**: 点击"一键启动"按钮
2. **图像切换**: 使用RGB/深度图像切换按钮
3. **开始探索**: 点击"开始探索"按钮
4. **查看截图**: 在人员位置表格中点击"查看截图"
5. **停止系统**: 点击"停止程序"按钮

## 📁 项目结构

```
explore_system/
├── start.py                    # 主程序入口
├── dashboard.py               # 仪表盘UI组件
├── topics_subscriber.py       # ROS话题订阅器
├── topic_logger.py           # 日志记录器
├── ball_pose_tracker.py      # 目标位置跟踪器
├── build_executable.py       # 打包脚本
├── quick_install.sh          # 快速安装脚本
├── install_desktop_icon.sh   # 桌面图标安装脚本
├── topics_config.json        # 话题配置文件
├── my_config.rviz           # RViz配置文件
├── logo.png                 # 程序图标
├── resource/                # 资源文件目录
├── 打包安装说明.md           # 详细安装说明
└── README.md               # 项目说明文件
```

## 🔧 配置说明

### 话题配置
编辑 `topics_config.json` 来配置ROS话题：
```json
{
    "camera": {
        "topic": "/image_converter/output_video",
        "msg_type": "sensor_msgs/Image"
    },
    "depth": {
        "topic": "/camera/depth/image_rect_raw",
        "msg_type": "sensor_msgs/Image"
    }
}
```

### 目标跟踪参数
编辑 `ball_pose_tracker.py` 顶部的全局参数：
```python
MIN_POSITIONS_FOR_PROCESSING = 3    # 最少处理帧数
DETECTION_TIMEOUT_SECONDS = 1.0     # 检测超时时间
POSITION_THRESHOLD_METERS = 1.5     # 位置阈值
```

## 📊 数据目录

程序运行时会创建以下目录：
- `ball_screenshots/`: 目标截图保存目录
- `screenshots/`: 一般截图保存目录
- `log/`: 系统日志文件目录

## 🐛 故障排除

### 常见问题

1. **程序无法启动**
   ```bash
   # 检查ROS环境
   source /opt/ros/noetic/setup.bash
   roscore &
   ```

2. **图像不显示**
   - 检查摄像头连接
   - 验证话题是否发布：`rostopic list`
   - 检查话题配置文件

3. **权限错误**
   ```bash
   chmod +x ~/drone_search_system/drone_search_system
   chmod +x ~/drone_search_system/run_drone_system.sh
   ```

4. **依赖缺失**
   ```bash
   pip3 install numpy opencv-python psutil PyQt5
   ```

### 调试模式
```bash
# 在终端中运行以查看详细输出
cd ~/drone_search_system
./drone_search_system
```

## 📝 开发说明

### 开发环境设置
```bash
# 安装开发依赖
pip3 install -r requirements.txt

# 设置ROS环境
source /opt/ros/noetic/setup.bash
```

### 代码结构
- `start.py`: 主程序，包含UI和系统控制逻辑
- `dashboard.py`: 自定义UI组件
- `topics_subscriber.py`: ROS话题订阅和数据处理
- `ball_pose_tracker.py`: 目标跟踪算法

### 打包新版本
```bash
# 修改代码后重新打包
python3 build_executable.py
```

## 📄 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

## 📞 支持

如果遇到问题，请：
1. 查看 [故障排除](#故障排除) 部分
2. 检查系统日志文件
3. 提交 Issue 描述问题

---

**版本**: v1.0  
**更新日期**: 2024年  
**兼容性**: Ubuntu 20.04 + ROS Noetic
