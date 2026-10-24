# A2W-ws — Livox Mid-360 + Point-LIO (ROS 2 Jazzy)

Mid-360 激光雷达 + Point-LIO 紧耦合里程计的工作空间。工作空间原名 `ros_ws`，已改名
`A2W-ws`（路径 `~/Projects/A2W-ws`）。

## 硬件与网络

| 项目 | 值 |
|---|---|
| 雷达 | Livox Mid-360，IP **192.168.1.179** |
| 主机网卡 | `enp5s0`，IP **192.168.1.50/24** |
| 网络配置 | NetworkManager「有线连接 1」（manual，192.168.1.50/24，未配网关） |
| 默认路由 | 走 WiFi（`wlp4s0`），雷达网卡不抢默认路由 |

驱动 `mid360_driver` **自动识别雷达 IP**，不需要在配置里写雷达地址；唯一要配对的是
`src/mid360_driver/config/param.yaml` 里的 `host_ip` —— 它必须是本机接雷达网卡的地址
（现在是 `192.168.1.50`）。换机器 / 换网卡时这里必须同步改，否则收不到数据。

驱动是**被动**的：只在本机监听 UDP **56301**（点云）/ **56401**（IMU），不向雷达下发任何配置。
因此雷达必须事先被配置成往本机 `host_ip` 推流（见「已知注意事项」）。本机实测：真实雷达
（`192.168.1.179`）会往 `192.168.1.50` 推流；换网卡 / 主机后按上面的规则重配。

```bash
# 检查链路 / 网络
ip -br addr show enp5s0
ping -c 3 192.168.1.179
nmcli connection show --active
```

## 依赖安装

```bash
sudo apt install -y ros-jazzy-pcl-ros
sudo apt install -y ros-jazzy-octomap-server ros-jazzy-octomap-msgs ros-jazzy-octomap-rviz-plugins
rosdep check --from-paths src --ignore-src
```

`octomap-*` 只有 `mid360_bringup` 用到（3D 占据栅格），不装也能跑驱动 + point_lio。

`mid360_driver` 上游需要 `libasio-dev`；本仓库的本地补丁 **M1** 已改用系统自带的
Boost.Asio（`src/mid360_driver/src/asio_compat.hpp`），**不需要**单独安装 Asio。
`libapr1-dev` / `libaprutil1-dev` / `livox_lidar_sdk` 都是旧 `livox_ros_driver2` 的依赖，
已随驱动一起移除。

## 编译

```bash
cd ~/Projects/A2W-ws
source /opt/ros/jazzy/setup.zsh      # 本机默认 shell 是 zsh，不要用 setup.bash
colcon build
source install/setup.zsh
```

> 若 `colcon build` 报 `ModuleNotFoundError: No module named 'catkin_pkg'`，说明 `which python3`
> 指向了 uv/venv 的 Python（本机 `~/.local/bin/python3` 就是 uv 3.14）。用
> `PATH=/usr/bin:$PATH colcon build`，或
> `colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`。

## 运行

新驱动直接发布 `sensor_msgs/PointCloud2`，point_lio 订阅的也是 PointCloud2
（`src/point_lio_ros2/src/laserMapping.cpp:781`），两边类型天然一致。旧版的
`xfer_format` / `CustomMsg` / `livox_ros_driver2` 说明已全部废弃（历史见
`docs/PATCHES.md`）。

```bash
# 终端 1 —— 驱动
source ~/Projects/A2W-ws/install/setup.zsh
ros2 launch mid360_driver mid360_driver.launch.py

# 终端 2 —— 建图
source ~/Projects/A2W-ws/install/setup.zsh
ros2 launch point_lio mapping_mid360.launch.py rviz:=false
```

驱动可执行文件是 `mid360_driver_node`（launch 里节点名 `mid360_driver`，包 `mid360_driver`），
参数在 `src/mid360_driver/config/param.yaml`：`lidar_topic: /livox/lidar`、`imu_topic: /livox/imu`、
`host_ip: 192.168.1.50`。改 `host_ip` 只影响本机监听地址；雷达侧的目的 IP 仍要单独配
（驱动不会代劳）。

### 一键启动（驱动 + point_lio + octomap + rviz）

```bash
source ~/Projects/A2W-ws/install/setup.zsh
ros2 launch mid360_bringup mid360.launch.py
```

这一个命令把四样都拉起来，rviz 单窗口显示 octomap 体素 + LIO 点云 + path（配置在
`src/mid360_bringup/rviz/octomap_mid360.rviz`）。一键模式下 rviz 由
`mid360.launch.py` 直接启动，关闭它用 `show_rviz:=false`（不用 `rviz:=false`
——launch 的 include 会把子 launch 的 `rviz` 参数写进共享配置，连父级一起改掉，
详见 `mid360.launch.py` 头注释）。`resolution=` / `max_range=` 这类调参请在
`octomap_mid360.launch.py` 那层做。想手动分部启动或单独调参，
照上面驱动 + 建图两终端的方式。

> 若上一轮是 Ctrl-C 退出的，启动前先 `bash src/mid360_bringup/scripts/cleanup.sh`
> 清残留进程（进程名已改成 `mid360_driver_node`），否则可能报
> `Frame [camera_init] does not exist`（原因见 `cleanup.sh` 顶部注释）。

### 话题

| 话题 | 类型 | 频率 |
|---|---|---|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | 10 Hz |
| `/livox/imu` | `sensor_msgs/Imu` | ~200 Hz |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | 10 Hz |
| `/aft_mapped_to_init` | `nav_msgs/Odometry` | 里程计（**不叫 `/Odometry`**） |
| `/Laser_map`, `/cloud_effected`, `/path` | | |
| `/backend/path` | `nav_msgs/Path` | 后端优化轨迹(map 系) |
| `/backend/odom` | `nav_msgs/Odometry` | 后端实时位姿(map→body) |
| `/backend/map_points` | `sensor_msgs/PointCloud2` | 后端优化点云(map 系) |
| `/backend/loop_markers` | `visualization_msgs/MarkerArray` | 回环边 + 关键帧 |
| `/backend/status` | `diagnostic_msgs/DiagnosticArray` | 关键帧/回环/优化耗时 |

`odom_only:=true` 时里程计改名 `/odom_corrected`。

`/livox/lidar` 的点云布局由 `mid360_driver` 固定：`x` / `y` / `z` / `intensity` 为
`float32`（offset 0/4/8/12），`timestamp` 为 `float64`（offset 16），单位**秒**
（绝对 Unix 时间，不是旧驱动的 ns）；`point_step=24`，**没有** `tag` / `line` 字段。
`header.stamp` 取该帧内**最早点**的时刻。IMU 的 `header.stamp` 同样是绝对 Unix 秒。

### 坐标系

| frame | 谁发的 | 说明 |
|---|---|---|
| `camera_init` | point_lio（odom 父帧） | 世界系 / 建图原点，重力对齐 |
| `body` | point_lio（tf 子帧） | IMU 机体系。重力方向为 -z |

point_lio 只发这两帧之间的一个 tf（`camera_init → body`），没有 URDF。

### 验证

仓库外（`/tmp`）放一次性验证脚本，不污染仓库。两种方式：

1. **fake UDP（无雷达）**：向 `host_ip:56301` / `host_ip:56401` 发构造好的 UDP 包，
   检查驱动能否解析出点云 / IMU，并核对 `point_step=24`、`timestamp` 为秒、
   `header.stamp` 为帧内最早点。改完驱动代码后的快速回归用这个。
2. **真实雷达**：`ros2 topic hz /livox/lidar`（应 10 Hz）、
   `ros2 topic hz /livox/imu`（应约 200 Hz），再用 `ros2 topic echo --once`
   或小脚本核对字段与时间戳。

   本机实测（真实雷达，2026-10-24）：`/livox/lidar` ≈9.9–10.2 Hz、约 1.9 万点/帧；
   `/livox/imu` ≈200–205 Hz；`header.stamp` 与帧内最早点差 0 s；IMU `|a|≈9.77 m/s²`。

旧版 `/tmp/verify_mid360.sh` / `/tmp/e2e_test.sh` 是 `livox_ros_driver2` 时代的，
已随驱动替换作废。

## 3D 占据栅格（octomap）

把 point_lio 的点云变成 octomap 三维占据栅格（可再投影出 2D 栅格给 nav2）。

```bash
# 方式一（推荐）：一键启动全部
ros2 launch mid360_bringup mid360.launch.py

# 方式二：分部启动，octomap 单独跑
# 终端 1、2 先把驱动 + point_lio 起起来（见上一节，point_lio 用 rviz:=false），然后：
source ~/Projects/A2W-ws/install/setup.zsh
ros2 launch mid360_bringup octomap_mid360.launch.py rviz:=true
```

说明：octomap 的 rviz 是**单窗口**（自带 `rviz/octomap_mid360.rviz`，同时显示
`/octomap_point_cloud_centers`、`/cloud_registered`、`/path`、TF），所以
point_lio 那边用 `rviz:=false` 关掉它自己的 rviz，别开两个窗口。

> 3D 栅格为什么用点云显示：rviz 的 octomap 插件（`octomap_rviz_plugins`，
> Jazzy 2.3.x 里叫 `OccupancyGrid`）在本机加载报
> `undefined symbol: octomap::OcTreeStamped` —— 插件 2.1.0 与系统
> `liboctomap` 1.9.7 ABI 不匹配，是发行包打包问题，不是我们的配置问题。
> 所以默认用 `/octomap_point_cloud_centers`（占据体素中心的点云，标准
> PointCloud2 display，零插件依赖）；装到匹配版本的插件后可换回
> `octomap_rviz_plugins/OccupancyGrid` 订阅 `/octomap_full` 看体素方块。

⚠️ 一键启动（`mid360.launch.py`）时 rviz 是父 launch 直接起的，不带
`resolution=` / `max_range=` 透传；要调这两个参数用分部启动
（`ros2 launch mid360_bringup octomap_mid360.launch.py resolution:=0.2 max_range:=30.0`）。

### 为什么是 `/cloud_registered_body` 而不是 `/cloud_registered`

这是接 octomap 最容易踩的坑。octomap_server（ROS 2 分支源码
`octomap_server.cpp` / `insertCloudCallback`）把射线起点算成：

```cpp
// sensor_to_world = lookupTransform(world_frame_id_, cloud->header.frame_id, ...)
const auto & t = sensor_to_world_transform_stamped.transform.translation;
insertScan(tf2::Vector3{t.x, t.y, t.z}, pc_ground, pc_nonground);
```

即：**射线起点就是“点云所在坐标系”在 `frame_id` 下的平移量**。

- 喂 `/cloud_registered`（header.frame_id 已经是 `camera_init` == `frame_id`）→
  查找出来是单位变换 → 射线全从**世界原点**射出。占据体素依旧是对的（rviz 里
  地图看着很正常），但自由空间被整片刷错，远处墙面会被从背面清空——2D 投影和
  后续导航都会遇到鬼影墙。
- 喂 `/cloud_registered_body`（header.frame_id = `body`）→ 射线起点 = 机器人当前
  位置，这才是传感器模型该有的样子。

所以 `mid360.yaml` 里把 `publish.scan_bodyframe_pub_en` 改成了 `true`，
并将 `odom_child_frame_id` 从上游默认的 `aft_mapped` 改为 `body` ——
因为 `/cloud_registered_body` 的 frame_id 在 `laserMapping.cpp:593` 是硬编码的
`"body"`，不统一的话 octomap_server 会直接卡在
`Message Filter dropping message: frame 'body'` / `Transform error`。

### 输出

| 话题 | 类型 | 说明 |
|---|---|---|
| `/octomap_full` | `octomap_msgs/Octomap` | 完整栅格（rviz 的 Octomap display 订阅这个） |
| `/octomap_binary` | `octomap_msgs/Octomap` | 无损压缩版 |
| `/octomap_point_cloud_centers` | `sensor_msgs/PointCloud2` | 占据体素中心点云 |
| `/occupied_cells_vis_array` | `MarkerArray` | 按 octree 深度分组的立方体 |
| `/projected_map` | `nav_msgs/OccupancyGrid` | 2D 投影，给 nav2 / costmap |

服务：`/octomap_server/octomap_full`、`/octomap_server/octomap_binary`（配合
`ros2 run octomap_server octomap_saver` 存盘）、`/octomap_server/clear_bbox`。

### 调参

都在 `src/mid360_bringup/config/octomap_mid360.yaml`，常用两个可以直接命令行覆盖：

```bash
ros2 launch mid360_bringup octomap_mid360.launch.py resolution:=0.2 max_range:=30.0
```

- `resolution` 体素边长。0.1 m 够用；调大省内存/CPU。
- `sensor_model.max_range` 射线截断距离。默认 20 m；
  Mid-360 本身量程 100 m，不截断会让 10 Hz 的射线投射量爆炸、地图无限膨胀。
- `occupancy_min_z/max_z` 只有这一高度带内的体素进 2D 投影。

### 性能与漂移

- 开启 octomap_server 后多了一个吃 10 Hz 稠密点云的 C++ 节点。若 CPU 吃紧，
  先把 `sensor_model.max_range` 调到 10~15 m，或把 point_lio 的
  `point_filter_num` / `filter_size_surf` 调大。
- **point_lio 本身没有回环**：纯 LIO 长时间绕回来会有漂移，octomap 会把同一面墙
  叠成两层。本工作区用 `point_lio_backend` 补上了回环检测 + 位姿图（默认开启，见
  「后端优化」一节）；`backend:=false` 时回到原来的纯 LIO 行为。
- 跑完想存地图：`ros2 run octomap_server octomap_saver -f ~/map`（需先让
  节点发布 `/octomap_full`；另 `.bt` 可用 `octovis` 看）。

## 后端优化（回环检查 + 位姿图）

point_lio 只做前端 LIO，没有回环，长距离绕回来必然漂移。`point_lio_backend` 是独立进程：
订阅前端输出、检测回环、跑 Ceres 位姿图，把世界系统一到新的 `map` 系，并通过 TF 把纠偏量
实时作用到 octomap / nav2。**不改 point_lio 源码**。

```
map --(backend, 20 Hz 纠偏)--> camera_init --(point_lio)--> body
```

| 项 | 值 |
|---|---|
| 回环检测 | Scan Context（20 环 × 60 扇区）+ GICP 粗到细校验 |
| 优化 | Ceres 6-DOF 位姿图（Huber + 残差门控剔外点） |
| 纠偏 | `T_map_camera_init = T_map_body · (T_camera_init_body)⁻¹`，一阶平滑 1.5 s |
| 输出 | `/backend/path`、`/backend/odom`、`/backend/map_points`、`/backend/loop_markers`、`/backend/status`、TF `map→camera_init` |

### 启动

```bash
# 默认开启后端（octomap / rviz 自动切到 map 系）
ros2 launch mid360_bringup mid360.launch.py

# 关掉后端，回到改造前行为（octomap 用 camera_init）
ros2 launch mid360_bringup mid360.launch.py backend:=false

# 只调试后端（前端另开终端）
ros2 launch point_lio_backend backend_mid360.launch.py
```

### 验收（绕一圈）

1. 起步后 `ros2 topic echo /backend/status`：`keyframes` 在增长、`loops=0`。
2. 绕回起点附近，`/backend/loop_markers` 出现回环边，`loops` 加一，
   `/backend/path` 在起点处闭合。
3. `ros2 run tf2_ros tf2_echo map camera_init`：纠偏量在 1~2 s 内平滑收敛。
4. octomap（map 系）里的重影会在继续观察几秒后被射线清掉；要立刻得到干净地图，
   优化收敛后重启 octomap / 重新建图。

### 调参

唯一真相源是 `src/point_lio_backend/config/backend_mid360.yaml`，常用项：

- `keyframe.min_distance` / `min_rotation_deg`：关键帧密度。调小 → 回环更容易、CPU 更高。
- `scan_context.sc_distance_threshold`（0.35）：调小更保守，重复走廊建议 0.30。
- `icp.max_fitness` / `min_inlier_ratio`：回环几何校验门限。
- `output.correction_tau`：纠偏平滑时间常数；0 = 回环瞬间直接跳。
- `runtime.pending_clouds`：后端排队上限，CPU 吃紧时保持 2。

### 已知限制

- **现有 `PCD/scans.pcd` 不能事后优化**：它只有合并后的点，没有逐帧位姿。后端必须
  建图时在线跑；要复算只能重跑一遍建图。
- **重复走廊可能误检**：SC + GICP + 修正量门限已经比较保守，仍建议把
  `sc_distance_threshold` 调到 0.30 并看 `/backend/loop_markers`。
- **漂移 > 5 m 可能漏检**：GICP 初值来自前端相对位姿，漂移过大时宁可安全失败
  （不优化），也不会把地图拉歪。
- **后端重启 = 图丢失**：重启后端会重新锚定 `map`，octomap 需一并重启。
- 后端崩溃时 `map→camera_init` TF 消失，octomap/rviz 会报 TF 错误；此时用
  `backend:=false` 或重启整套。

### 测试

```bash
# 单元测试（Scan Context / 位姿图 / 关键帧）
PATH=/usr/bin:$PATH colcon test --packages-select point_lio_backend --event-handlers console_direct+

# 合成回环（不需要真实雷达；先用 PCD/scans.pcd 当世界）
export ROS_DOMAIN_ID=77
source install/setup.bash
python3 src/point_lio_backend/test/integration/synthetic_loop_replay.py --mode loop
python3 src/point_lio_backend/test/integration/synthetic_loop_replay.py --mode straight
```

### 与上游的差异

本工作空间对两个上游包做过实质改造，这是本仓库存在的主要价值：

### mid360_driver
上游：`https://github.com/Yancey2023/mid360_driver`（upstream/main `e2ba0ca`，MIT，作者 Yingjie Huang）。
本仓库 vendor 的基线是本地 fork `xiaozhuABCD1234/mid360_driver` 的 `4adf8df`（`add mid70 driver`，
即 upstream/main + Mid-70 支持；多出的 `mid70_driver_node` 不影响 Mid-360 通路）。

- 换掉旧 `livox_ros_driver2` 的原因：不依赖 Livox-SDK2，直接实现 UDP；自动识别雷达 IP；
  直接发 `sensor_msgs/PointCloud2`，point_lio 不需要 CustomMsg；依赖更轻。
- **M1（本地补丁）**：本机没有 `libasio-dev` 且无法 sudo，驱动改用 Boost.Asio。
  新增 `src/mid360_driver/src/asio_compat.hpp`，源码里的 `#include <asio.hpp>` 改为
  `#include "asio_compat.hpp"`，`package.xml` 的依赖 `asio` 改为 `boost`。
- `config/param.yaml`：`host_ip` 固定为本机网卡地址 `192.168.1.50`；`lidar_topic` /
  `imu_topic` 保持 `/livox/lidar`、`/livox/imu`。
- 驱动是被动接收端（UDP 56301/56401），不向雷达下发目的 IP。
- 话题、点云布局、实测频率见上文「话题」节；实测通过（见提交记录）。

### point_lio_ros2
上游：`https://github.com/dfloreaa/point_lio_ros2` @ `a8e2d0d`

- `src/preprocess.cpp` / `src/preprocess.h` —— 新增 `sensor_msgs/PointCloud2`
  输入路径：`livox_handler` 用 `sensor_msgs::PointCloud2ConstIterator` **按字段名**取
  `x/y/z/intensity(float32)` 与 `timestamp(float64)`，不再依赖 PCL 点结构体布局；
  新布局没有 `tag/line`。帧内时间 = `timestamp - header.stamp`，`timestamp` 是绝对
  Unix **秒**（`timestamp_unit=0`），换算成 ms 作为 `curvature`。取代了原 CustomMsg 路径。
- 删除 `.gitmodules` —— 上游遗留的死配置：声明 `ikd-Tree` 与 `include/IKFoM` 为子模块，
  但实际两者均已 vendored 成普通文件（无 gitlink），且 `ikd-Tree` 声明路径与实际的
  `include/ikd-Tree/` 不符。扁平化后已无子模块关系，保留会误导并可能被
  `git submodule update` 覆写掉 vendored 代码。
- `config/mid360.yaml` —— 为配合 octomap：`odom_child_frame_id` 由上游默认的
  `aft_mapped` 改为 `body`（与 `/cloud_registered_body` 硬编码的 frame_id 对齐，否则
  octomap_server 拿不到 tf）；`publish.scan_bodyframe_pub_en` 改为 `true`。
  `timestamp_unit` 由 `3`（ns）改为 `0`（秒，P9），与 `mid360_driver` 的点时间一致；
  `mapping.acc_norm` 由 `1.0`（g）改为 `9.81`（m/s²，P10），因为 `mid360_driver` 发布 IMU
  前已把原始 g 值乘 9.81（旧 `livox_ros_driver2` 直接发 g）。

### mid360_bringup（**本仓库新增包**，非上游）

只放集成用的 launch / config，无编译产物。目前包含 `launch/octomap_mid360.launch.py`
（point_lio → octomap 三维占据栅格，见 README「3D 占据栅格」一节）。
`launch/mid360.launch.py` 里的驱动已改为 include `mid360_driver`，`package.xml` 的
`exec_depend` 也从 `livox_ros_driver2` 换成 `mid360_driver`；`scripts/preflight.sh` /
`cleanup.sh` 也同步适配（清理进程名 `mid360_driver_node`）。

### 已删除的 livox_ros_driver2

旧的 `livox_ros_driver2` 已从 `src/` 删除（删除前 main HEAD=`4448841`）。本机缺
`libaprutil1-dev`，它在本工作空间已无法构建；旧补丁 P3（`xfer_format` 1→0）随之作废。
需要查旧实现（例如临时用旧驱动给雷达配目的 IP）时可恢复：

```bash
git checkout 4448841 -- src/livox_ros_driver2
```

恢复后要自己解决 `libapr1-dev` / `libaprutil1-dev` / `livox_lidar_sdk` 依赖，不保证能编译。

### 上游历史备份

point_lio 的独立 `.git` 曾移出仓库外（未销毁），旧路径为
`~/ros_ws-upstream-git/point_lio_ros2.git`（本机当前已不存在；需要历史时按上游 URL
重新 clone）。mid360_driver 的上游对照仓库在本机 `~/Projects/mid360_driver`，
`git remote -v` 里 `upstream` 指向 `Yancey2023/mid360_driver`。

```bash
# 若旧备份还在：
git --git-dir=~/ros_ws-upstream-git/point_lio_ros2.git log --oneline
# mid360_driver 上游对照：
git -C ~/Projects/mid360_driver log --oneline
```

## 已知注意事项

- **雷达目的 IP 要预先配好**。`mid360_driver` 是被动接收，不会向雷达下发目的地址；
  它只在 `host_ip` 上监听 UDP 56301/56401。换主机 / 网卡后如果 `/livox/lidar` 没数据：
  ① 确认 `src/mid360_driver/config/param.yaml` 的 `host_ip` 已改成新网卡地址；
  ② 用 Livox Viewer（或临时恢复旧驱动，见上一节）把雷达的目的 IP 指到该地址。
  雷达 IP 本身不用配（驱动自动识别）。
- 本机默认 shell 是 **zsh**。ROS 的 `setup.bash` 是 bash 专用模板，zsh 下会因
  `BASH_SOURCE` 为空而去当前目录找 `setup.sh` 报 `127`，请一律用 `setup.zsh`。
- 若 `pointlio_mapping` 启动即报 `symbol lookup error: libpcl_io.so... undefined symbol:
  libusb_set_option`，那是 `~/.bashrc` 把 `/opt/MVS/lib/64` 放进 `LD_LIBRARY_PATH`
  （相机 SDK 自带的旧 `libusb-1.0.so.0` 抢先被加载）导致的。`mapping_mid360.launch.py`
  已加 **P11** 自动剔除该路径并打印提示；若你直接跑 `pointlio_mapping` 二进制，仍需自己
  `export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | tr ':' '\n' | grep -v /opt/MVS | paste -sd:)`。
  这个错误会让 point_lio 进程 exit 127，rviz 紧接着报 `Frame [camera_init] does not exist`。
- `src/point_lio_ros2/image/` 共 246MB 演示 GIF（最大单个 89.6MB），已从 `.gitignore` 中排除。
- `src/point_lio_ros2/Log/{pos_log,mat_out,imu_pbp}.txt` 是运行时被清零覆写的产物，已忽略。
- `src/point_lio_ros2/include/IKFoM/` 下 3 个 `.vscode/settings.json`（上游跟踪的 IDE 配置）
  被本仓库的 `.vscode/` 全局规则排除，不影响编译。
- `src/point_lio_ros2/PCD/temp.txt` 是上游用来占位的空文件（PCL 的 `savePCDFileBinary`
  不自建目录）。**LOCAL PATCH P8 之后落盘改成相对路径**：目录是**运行 `ros2 launch` 的那个
  目录**下的 `PCD/`（通常是本工作空间根目录），不是 `src/point_lio_ros2/PCD/`。
  那个占位文件在新位置已无用 —— 新目录由节点启动时自己 `mkdir`（已在就不动，
  建不了会打印 `[pcd_save] mkdir PCD 失败(...)`）。
