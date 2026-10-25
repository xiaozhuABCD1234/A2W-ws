# A2W-ws

Livox Mid-360 + Point-LIO 的 ROS 2 Jazzy 工作空间：从雷达取数、实时 LIO 建图，到在线回环优化、3D 占据栅格与离线点云查看。

## 功能

| 包 | 功能 |
|---|---|
| `mid360_driver` | 直接通过 UDP 接收 Mid-360 点云和 IMU，发布 `sensor_msgs/PointCloud2` 与 `sensor_msgs/Imu`；自动识别雷达 IP |
| `point_lio_ros2` | Point-LIO 紧耦合里程计与建图；输出配准点云、里程计、轨迹，并可保存 PCD |
| `point_lio_backend` | 在线回环检测（Scan Context + GICP）与 Ceres 位姿图优化；发布 `map` 系纠偏 TF、优化轨迹和校正后的地图 |
| `mid360_bringup` | 全栈一键启动、octomap 3D 占据栅格、rviz 配置、启动自检、TF 诊断、离线 PCD 查看 |

数据流：

```
Mid-360 --UDP--> mid360_driver --> point_lio_ros2 --> point_lio_backend
                                        |                    |
                                        +--> octomap / rviz <+
```

## 环境与依赖

- ROS 2 Jazzy
- 基础依赖：

```bash
sudo apt install ros-jazzy-pcl-ros libceres-dev
```

- 3D 占据栅格（可选，不装也能跑核心链路）：

```bash
sudo apt install ros-jazzy-octomap-server ros-jazzy-octomap-rviz-plugins
```

## 编译

```bash
cd ~/Projects/A2W-ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

若使用 zsh，将 `setup.bash` 换成 `setup.zsh`。

## 使用

### 1. 雷达网络

驱动是被动接收端，只在本地监听 UDP **56301**（点云）和 **56401**（IMU），不会修改雷达配置。使用前确认：

- 接雷达的网卡地址等于 `src/mid360_driver/config/param.yaml` 中的 `host_ip`。
- 雷达的目的 IP 也指向同一地址。

雷达自身 IP 由驱动自动识别。链路检查：

```bash
ip -br addr
sudo tcpdump -ni <网卡名> udp port 56301
```

### 2. 一键启动

```bash
source ~/Projects/A2W-ws/install/setup.bash
ros2 launch mid360_bringup mid360.launch.py
```

默认启动驱动、Point-LIO、后端、octomap 和 rviz，rviz 使用 `map` 系。

| 参数 | 默认 | 说明 |
|---|---|---|
| `show_rviz` | `true` | 是否启动 rviz |
| `backend` | `true` | 是否启用回环与位姿图；关闭后回到 `camera_init` 系 |
| `use_octomap` | `true` | 是否构建 3D 占据栅格 |
| `pcd_save` | 留空 | 是否落盘建图点云；留空时跟随 `point_lio` 配置（当前开启） |

示例：

```bash
ros2 launch mid360_bringup mid360.launch.py show_rviz:=false
ros2 launch mid360_bringup mid360.launch.py backend:=false
```

系统未安装 octomap 时，一键启动会自动跳过占据栅格，其余功能照常运行。

### 3. 分步启动

```bash
# 终端 1：驱动
ros2 launch mid360_driver mid360_driver.launch.py

# 终端 2：Point-LIO
ros2 launch point_lio mapping_mid360.launch.py rviz:=false

# 终端 3：后端（可选，一键启动已包含）
ros2 launch point_lio_backend backend_mid360.launch.py

# 终端 4：octomap + rviz（可选）
ros2 launch mid360_bringup octomap_mid360.launch.py rviz:=true
```

### 4. 地图文件

正常退出（Ctrl-C）时，建图结果写到运行目录下的 `PCD/`：

| 文件 | 坐标系 | 说明 |
|---|---|---|
| `PCD/scans.pcd` | `camera_init` | 前端建图结果，不含回环校正 |
| `PCD/backend_map.pcd` | `map` | 后端回环优化后的地图 |

### 5. 离线查看 PCD

不需要雷达或建图节点：

```bash
ros2 launch mid360_bringup pcd_view.launch.py
ros2 launch mid360_bringup pcd_view.launch.py pcd:=./PCD/backend_map.pcd
```

### 6. 常用工具

```bash
bash src/mid360_bringup/scripts/cleanup.sh       # 清理残留进程
bash src/mid360_bringup/scripts/check_tf.sh      # 检查 TF、话题和帧 id
bash src/mid360_bringup/scripts/preflight.sh --runtime  # 配置/运行状态检查
```

## 主要话题与坐标系

| 话题 | 类型 | 说明 |
|---|---|---|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | 雷达点云，10 Hz |
| `/livox/imu` | `sensor_msgs/Imu` | IMU，约 200 Hz |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | 配准到世界系的点云 |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | 机体系点云，octomap 使用 |
| `/aft_mapped_to_init` | `nav_msgs/Odometry` | 前端里程计 |
| `/backend/path` | `nav_msgs/Path` | 后端优化轨迹 |
| `/backend/map_points` | `sensor_msgs/PointCloud2` | 后端优化点云 |
| `/backend/status` | `diagnostic_msgs/DiagnosticArray` | 关键帧、回环与耗时 |

TF 链：

```
map --(backend)--> camera_init --(Point-LIO)--> body
```

- `camera_init`：前端世界系，重力对齐。
- `body`：IMU 机体系。
- `map`：后端优化后的世界系；关闭后端时不存在。

## 目录结构

```
src/
  mid360_driver/      雷达驱动
  point_lio_ros2/     Point-LIO 前端
  point_lio_backend/  回环与位姿图后端
  mid360_bringup/     启动文件、octomap、rviz、工具脚本
docs/                 设计、验收与补丁文档
```

## 文档

- [docs/PATCHES.md](docs/PATCHES.md)：本地补丁与上游差异
- [docs/BACKEND_SPEC.md](docs/BACKEND_SPEC.md)：后端接口与设计
- [docs/BACKEND_ACCEPTANCE.md](docs/BACKEND_ACCEPTANCE.md)：后端测试与验收数据
- [docs/LOCALIZATION.md](docs/LOCALIZATION.md)：重定位方案调研（尚未实现）

## 已知边界

- 当前只做在线建图与回环，尚未实现先验地图重定位。
- 后端不保存历史图，重启后端会重新锚定 `map`。
- 重复走廊等相似场景仍可能产生误回环，可通过后端配置收紧阈值。
- 回环优化只对在线运行有效；前端 `scans.pcd` 不能事后优化。

## 上游与许可

- `mid360_driver`：[Yancey2023/mid360_driver](https://github.com/Yancey2023/mid360_driver)（MIT）
- `point_lio_ros2`：[dfloreaa/point_lio_ros2](https://github.com/dfloreaa/point_lio_ros2)
- `point_lio_backend`、`mid360_bringup`：本仓库新增
