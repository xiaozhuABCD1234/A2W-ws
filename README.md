# nav-ws

Mid-360 建图 + 位姿图后端工作空间(ROS 2 Jazzy)。

## 常用命令(just)

```bash
just              # 列出全部命令
just build        # colcon 编译(Release)
just run          # 一键启动:preflight + 驱动/point_lio/后端/rviz
just octomap      # octomap 单独启动
just backend      # 后端位姿图单独启动
just check-tf     # TF/帧 id 诊断
just preflight    # 启动前配置自洽性检查
just cleanup      # 清理残留驱动进程
just record       # 录制 ros2 bag(/livox/lidar + /livox/imu),Ctrl+C 结束
just realsense-setup  # 安装 D435i udev 规则(一次性)
just test-backend # 后端 gtest
```

## 依赖

```bash
sudo apt install ros-jazzy-pcl-ros ros-jazzy-pcl-conversions ros-jazzy-octomap-server ros-jazzy-octomap-rviz-plugins
```

- GTSAM 4.2:/usr/local 源码安装(apt 无对应包),位姿图依赖
- D435i:`sudo apt install ros-jazzy-realsense2-camera`,再跑 `just realsense-setup`
