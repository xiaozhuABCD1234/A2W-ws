# ros_ws — Livox Mid-360 + Point-LIO (ROS 2 Jazzy)

Mid-360 激光雷达 + Point-LIO 紧耦合里程计的工作空间。

## 硬件与网络

| 项目 | 值 |
|---|---|
| 雷达 | Livox Mid-360,IP **192.168.1.179** |
| 主机网卡 | `enx0c3d5ec81ff1` (USB 网口),IP **192.168.1.5/24** |
| 网络配置 | NetworkManager 连接 `mid360`,持久化于 `/etc/netplan/90-NM-<uuid>.yaml` |
| 已配 `never-default` | 是 —— 避免抢走 WiFi 默认路由 |

雷达 IP 写在 `src/livox_ros_driver2/config/MID360_config.json` 的 `lidar_configs[0].ip`;
`host_net_info` 的四个 IP 必须与主机网卡实际地址一致。

```bash
# 检查链路 / 网络
ip -br addr show enx0c3d5ec81ff1
ping -c 3 192.168.1.179
nmcli connection show --active
```

## 依赖安装

```bash
sudo apt install -y ros-jazzy-pcl-ros libapr1-dev libaprutil1-dev
rosdep check --from-paths src --ignore-src
```

还需系统级安装 `livox_lidar_sdk`(提供 `liblivox_lidar_sdk_shared.so` 到 `/usr/local/lib`)。

## 编译

```bash
cd ~/Projects/ros_ws
source /opt/ros/jazzy/setup.zsh      # 本机默认 shell 是 zsh,不要用 setup.bash
colcon build
source install/setup.zsh
```

## 运行

`msg_MID360_launch.py` 的默认值已改为 `xfer_format=0`,无需再在命令行传参。

原因:`point_lio` 订阅的是 `sensor_msgs/msg/PointCloud2`
(`src/point_lio_ros2/src/laserMapping.cpp:781`),只有 `xfer_format=0` 才发这个类型;
上游默认的 `xfer_format=1` 发的是 `livox_ros_driver2/msg/CustomMsg`,
CustomMsg 那套代码在 point_lio 里已全部注释掉。ROS 2 对同名话题类型不匹配**不报错**,
只会静默收不到数据 —— 表现是 IMU 初始化能到 100%,但一直没有点云(此坑已踩过)。

```bash
# 终端 1 —— 驱动
source ~/Projects/ros_ws/install/setup.zsh
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 终端 2 —— 建图
source ~/Projects/ros_ws/install/setup.zsh
ros2 launch point_lio mapping_mid360.launch.py rviz:=false
```

### 话题

| 话题 | 类型 | 频率 |
|---|---|---|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | 10 Hz (`xfer_format=0`,已是默认值) |
| `/livox/imu` | `sensor_msgs/Imu` | 200 Hz |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | 10 Hz |
| `/aft_mapped_to_init` | `nav_msgs/Odometry` | 里程计(**不叫 `/Odometry`**) |
| `/Laser_map`, `/cloud_effected`, `/path` | | |

`odom_only:=true` 时里程计改名 `/odom_corrected`。

### 验证脚本

仓库外(临时)有两个验证脚本:`/tmp/verify_mid360.sh`(仅驱动)、
`/tmp/e2e_test.sh`(驱动 + point_lio 端到端)。

## 与上游的差异

本工作空间对两个上游包做过实质改造,这是本仓库存在的主要价值:

### livox_ros_driver2
上游:`https://github.com/Livox-SDK/livox_ros_driver2` @ `4a1def9`

- `CMakeLists.txt` —— 删除 ROS 1 分支改为 ROS 2 专用;增加 `CMP0144` 策略处理消除
  Ubuntu 24.04 / Jazzy 上的警告;`DISTRO_ROS` 未传参时回退到 `$ROS_DISTRO` 环境变量。
- `launch_ROS2/msg_MID360_launch.py` —— 原本所有参数是模块级常量、无法命令行覆盖;
  改为全部 `DeclareLaunchArgument`,可在命令行传入;并把 `xfer_format` 默认值
  由上游的 `1`(CustomMsg)改为 `0`(PointCloud2),与 point_lio 的订阅类型对齐,
  避免默认配置下静默无点云。
- `src/comm/pub_handler.cpp` —— 删除 1 行。
- `config/MID360_config.json` —— 雷达 IP 改为 `192.168.1.179`。
- `package.xml` —— 上游不跟踪(由 `build.sh` 从 `package_ROS2.xml` 生成);
  本仓库强制纳入,保证 clone 后可直接 `colcon build`。

### point_lio_ros2
上游:`https://github.com/dfloreaa/point_lio_ros2` @ `a8e2d0d`

- `src/preprocess.cpp` (+44) / `src/preprocess.h` (+24) —— 新增 `sensor_msgs/PointCloud2`
  输入路径:`livox_handler` 按 `x,y,z,intensity(float32),tag,line(uint8),timestamp(float64)`
  解析 `xfer_format=0` 的点云,以绝对 ns 时间戳减去帧头得到相对时间,
  再按 `timestamp_unit` 换算为 ms。取代了原 CustomMsg 路径。
- 删除 `.gitmodules` —— 上游遗留的死配置:声明 `ikd-Tree` 与 `include/IKFoM` 为子模块,
  但实际两者均已 vendored 成普通文件(无 gitlink),且 `ikd-Tree` 声明路径与实际的
  `include/ikd-Tree/` 不符。扁平化后已无子模块关系,保留会误导并可能被
  `git submodule update` 覆写掉 vendored 代码。

### 上游历史备份

两个包原本的 `.git` 已移出仓库外(未销毁),仍可查询历史用于 diff / 合并上游:

```bash
git --git-dir=~/ros_ws-upstream-git/livox_ros_driver2.git log --oneline
git --git-dir=~/ros_ws-upstream-git/point_lio_ros2.git log --oneline
```

## 已知注意事项

- 本机默认 shell 是 **zsh**。ROS 的 `setup.bash` 是 bash 专用模板,zsh 下会因
  `BASH_SOURCE` 为空而去当前目录找 `setup.sh` 报 `127`,请一律用 `setup.zsh`。
- `src/point_lio_ros2/image/` 共 246MB 演示 GIF(最大单个 89.6MB),已从 `.gitignore` 中排除。
- `src/point_lio_ros2/Log/{pos_log,mat_out,imu_pbp}.txt` 是运行时被清零覆写的产物,已忽略。
- `src/point_lio_ros2/include/IKFoM/` 下 3 个 `.vscode/settings.json`(上游跟踪的 IDE 配置)
  被本仓库的 `.vscode/` 全局规则排除,不影响编译。
- `src/point_lio_ros2/PCD/temp.txt` **必须保留**。PCL 的 `savePCDFileBinary` 不会自动创建
  目录,`pcd_save_en:=true` 时缺了 `PCD/` 目录会存盘失败;上游用这个空文件占位。
