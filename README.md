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
sudo apt install -y ros-jazzy-octomap-server ros-jazzy-octomap-msgs ros-jazzy-octomap-rviz-plugins
rosdep check --from-paths src --ignore-src
```

`octomap-*` 只有 `mid360_bringup` 用到(3D 占据栅格),不装也能跑驱动 + point_lio。

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

### 一键启动(驱动 + point_lio + octomap + rviz)

```bash
source ~/Projects/ros_ws/install/setup.zsh
ros2 launch mid360_bringup mid360.launch.py
```

这一个命令把四样都拉起来,rviz 单窗口显示 octomap + LIO 点云 + path(配置在
`src/mid360_bringup/rviz/octomap_mid360.rviz`)。想手动分部启动或单独调参,
照下面三终端的方式。

### 话题

| 话题 | 类型 | 频率 |
|---|---|---|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | 10 Hz (`xfer_format=0`,已是默认值) |
| `/livox/imu` | `sensor_msgs/Imu` | 200 Hz |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | 10 Hz |
| `/aft_mapped_to_init` | `nav_msgs/Odometry` | 里程计(**不叫 `/Odometry`**) |
| `/Laser_map`, `/cloud_effected`, `/path` | | |

`odom_only:=true` 时里程计改名 `/odom_corrected`。

### 坐标系

| frame | 谁发的 | 说明 |
|---|---|---|
| `camera_init` | point_lio(odom 父帧) | 世界系 / 建图原点,重力对齐 |
| `body` | point_lio(tf 子帧) | IMU 机体系。重力方向为 -z |

point_lio 只发这两帧之间的一个 tf(`camera_init → body`),没有 URDF。

### 验证脚本

仓库外(临时)有两个验证脚本:`/tmp/verify_mid360.sh`(仅驱动)、
`/tmp/e2e_test.sh`(驱动 + point_lio 端到端)。

## 3D 占据栅格(octomap)

把 point_lio 的点云变成 octomap 三维占据栅格(可再投影出 2D 栅格给 nav2)。

```bash
# 方式一(推荐):一键启动全部
ros2 launch mid360_bringup mid360.launch.py

# 方式二:分部启动,octomap 单独跑
# 终端 1、2 先把驱动 + point_lio 起起来(见上一节,point_lio 用 rviz:=false),然后:
source ~/Projects/ros_ws/install/setup.zsh
ros2 launch mid360_bringup octomap_mid360.launch.py rviz:=true
```

说明:octomap 的 rviz 是**单窗口**(自带 `rviz/octomap_mid360.rviz`,同时显示
`/octomap_full`、`/cloud_registered`、`/path`、TF),所以 point_lio 那边用
`rviz:=false` 关掉它自己的 rviz,别开两个窗口。rviz 里 Octomap 那个 display
对应 `octomap_rviz_plugins` 插件,不是默认的 PointCloud2。

### 为什么是 `/cloud_registered_body` 而不是 `/cloud_registered`

这是接 octomap 最容易踩的坑。octomap_server(ROS 2 分支源码
`octomap_server.cpp` / `insertCloudCallback`)把射线起点算成:

```cpp
// sensor_to_world = lookupTransform(world_frame_id_, cloud->header.frame_id, ...)
const auto & t = sensor_to_world_transform_stamped.transform.translation;
insertScan(tf2::Vector3{t.x, t.y, t.z}, pc_ground, pc_nonground);
```

即:**射线起点就是“点云所在坐标系”在 `frame_id` 下的平移量**。

- 喂 `/cloud_registered`(header.frame_id 已经是 `camera_init` == `frame_id`)→
  查找出来是单位变换 → 射线全从**世界原点**射出。占据体素依旧是对的(rviz 里
  地图看着很正常),但自由空间被整片刷错,远处墉面会被从背面清空——2D 投影和
  后续导航都会遇到鬼影墙。
- 喂 `/cloud_registered_body`(header.frame_id = `body`)→ 射线起点 = 机器人当前
  位置,这才是传感器模型该有的样子。

所以 `mid360.yaml` 里把 `publish.scan_bodyframe_pub_en` 改成了 `true`,
并将 `odom_child_frame_id` 从上游默认的 `aft_mapped` 改为 `body` ——
因为 `/cloud_registered_body` 的 frame_id 在 `laserMapping.cpp:593` 是硬编码的
`"body"`,不统一的话 octomap_server 会直接卡在
`Message Filter dropping message: frame 'body'` / `Transform error`。

### 输出

| 话题 | 类型 | 说明 |
|---|---|---|
| `/octomap_full` | `octomap_msgs/Octomap` | 完整栅格(rviz 的 Octomap display 訂这个) |
| `/octomap_binary` | `octomap_msgs/Octomap` | 无损压缩版 |
| `/octomap_point_cloud_centers` | `sensor_msgs/PointCloud2` | 占据体素中心点云 |
| `/occupied_cells_vis_array` | `MarkerArray` | 按 octree 深度分组的立方体 |
| `/projected_map` | `nav_msgs/OccupancyGrid` | 2D 投影,给 nav2 / costmap |

服务:`/octomap_server/octomap_full`、`/octomap_server/octomap_binary`(配合
`ros2 run octomap_server octomap_saver` 存盘)、`/octomap_server/clear_bbox`。

### 调参

都在 `src/mid360_bringup/config/octomap_mid360.yaml`,常用两个可以直接命令行覆盖:

```bash
ros2 launch mid360_bringup octomap_mid360.launch.py resolution:=0.2 max_range:=30.0
```

- `resolution` 体素边长。0.1 m 够用;调大省内存/CPU。
- `sensor_model.max_range` 射线截断距离。默认 20 m;
  Mid-360 本身量程 100 m,不截断会让 10 Hz 的射线投射量爆炸、地图无限膨胀。
- `occupancy_min_z/max_z` 只有这一高度带内的体素进 2D 投影。

### 性能与漂移

- 开启 octomap_server 后多了一个吃 10 Hz 稠密点云的 C++ 节点。若 CPU 吃紧,
  先把 `sensor_model.max_range` 调到 10~15 m,或把 point_lio 的
  `point_filter_num` / `filter_size_surf` 调大。
- **没有回环检测**:point_lio 是纯 LIO,长时间绕回来会有漂移,octomap 会把
  同一面墙叠成两层。短时间建图没问题,要长期一致得先上 FAST-LIO-SAM 那类回环。
- 跑完想存地图:`ros2 run octomap_server octomap_saver -f ~/map`(需先让
  节点发布 `/octomap_full`;另 `.bt` 可用 `octovis` 看)。

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
- `config/mid360.yaml` —— 为配合 octomap:`odom_child_frame_id` 由上游默认的
  `aft_mapped` 改为 `body`(与 `/cloud_registered_body` 硬编码的 frame_id 对齐,否则
  octomap_server 拿不到 tf);`publish.scan_bodyframe_pub_en` 改为 `true`。

### mid360_bringup(**本仓库新增包**,非上游)

只放集成用的 launch / config,无编译产物。目前包含 `launch/octomap_mid360.launch.py`
(point_lio → octomap 三维占据栅格,见 README「3D 占据栅格」一节)。

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
