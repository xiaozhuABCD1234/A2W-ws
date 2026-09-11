# 重定位(先验地图定位)调研与选型

> 场景:机器人**被搬动 / 断电重启**后开机,开机瞬间**完全不知道自己在先验地图里的位姿**
> (所谓"盲启动" global localization)。
>
> 本文只做调研与选型,不含代码改动。证据标注约定:
> **[实测]** = 在本工作区/本机跑过或读过源码;**[上游]** = 上游 README/文档原文;
> **[未验证]** = 结论来自上游或第三方描述,本机没验。

---

## 0. 一句话结论

**ROS2 生态里没有"装上就能盲启动"的成品**。可行做法是自备两样先验资产
(**点云 `map.pcd`** + **2D 占据栅格 `map.yaml/pgm`**),然后把流程拆成两段自己接:

```
候选生成(2D 栅格上的 BBS 分支定界) ──▶ 精配准 + 校验(NDT/GICP + fitness)──▶ 给初值/重播种
```

两条现实路径:

| 路径 | 代价 | 适用 |
|---|---|---|
| **A. 2D 路线**:`pointcloud_to_laserscan` + `nav2_amcl` | 最低(apt 直接装,AMCL 天生就是多假设粒子滤波) | 单层/多层但层高可分的地面机器人。**本篇推荐先走这条打通闭环** |
| **B. 3D 路线**:`lidar_localization_ros2` 的 opt-in 全局定位栈(BBS_2D + NDT_OMP) | 中(要自己编包 + 调试帧/TF) | 需要 3D 姿态、非平面场景、或已有 Nav2 3D 定位需求 |

工程上先反问一句:**是不是真有"任意位置开机"?** 如果实际部署里机器人总有固定停靠点
(充电桩、工位、AGV 站点),那"盲启动"是伪需求 —— 固定初值 + 局部配准就够了,
成本低一个量级。

---

## 1. 本机硬约束(先排除一批方案)

| 项 | 实测值 | 对选型的影响 |
|---|---|---|
| CPU | 11th Gen Intel i5-1135G7,**4 核 8 线程** | NDT_OMP 别按默认 4 线程起;`point_lio`(10 Hz)+ octomap + 定位节点会互相抢 |
| 内存 | 15 GiB(空闲约 11 GiB) | 全分辨率 `scans.pcd` 能放得下,但 `pcd_save.interval: -1` 是**全部帧一次性累加**,长测要改 interval |
| GPU | **无 NVIDIA**(`nvidia-smi` 不存在,`/dev/nvidia*` 无) | **直接排除所有 GPU 方案**(见 §4 的 3D-BBS) |
| OS / ROS | Ubuntu 24.04 + Jazzy,默认 shell 是 **zsh** | 装包时用 `install/setup.zsh`;上游脚本多为 bash,注意 `source /opt/ros/jazzy/setup.bash` 与 zsh 混用 |
| `vcs`(python3-vcstool) | **未安装** | 拉上游依赖前先 `sudo apt install python3-vcstool` |

已确认 apt 里可用的相关包 **[实测]**:

| 包 | 版本 |
|---|---|
| `ros-jazzy-nav2-amcl` | 1.3.12 |
| `ros-jazzy-nav2-map-server` | 1.3.12 |
| `ros-jazzy-pointcloud-to-laserscan` | 2.0.2 |

---

## 2. 先验资产:现在只有 1 样,缺 2 样

| 资产 | 在定位链路里的角色 | 现状 |
|---|---|---|
| **先验点云 `.pcd`** | NDT / GICP / BBS 的**配准源** | ❌ **缺**。`config/mid360.yaml:82` 的 `pcd_save.pcd_save_en` 还是 `false`(即 `docs/PATCHES.md` 的 D 项) |
| **2D 占据栅格** | 盲启动的候选生成(BBS_2D)、或 AMCL 的地图 | ⚠️ **可从 `/projected_map` 导出**,不额外建图 |
| `.bt` 3D 占据栅格 | 规划 / 避障 / 可视化 | ✅ `~/maps/mid360.bt`(0.2 m,506 KB) |

### 2.1 为什么 `.bt` 不能直接当配准源

- 它是 **0.2 m 体素中心的占据栅格**:没有 intensity,且八叉树内部节点不是真实表面
  → 配准精度天花板 = 体素边长(`resolution: 0.2`);
- 它是**给规划用的**语义(占据/空闲),配对时用不到空闲的那一半信息;
- **octomap 没有 CLI 反解工具** **[实测]**:`/usr/bin` 下没有任何 `octomap*`/`bt*` 可执行文件;
  `octomap_server_static_node` 加载 `.bt` 后**只提供 service**(`/octomap_binary`、`/octomap_full`),
  **不发任何点云话题**(`ros2 node info` 实测确认)。
  想从 `.bt` 反推点云只剩两条路:(a) 还在跑的 `octomap_server` 订阅 `/octomap_point_cloud_centers`
  落盘;(b) 自己调 `GetOctomap` service 解码(需要 octomap 的 Python 绑定,默认没有)。
  两条都是**降级**产物,不值得为它绕路。

### 2.2 另一个看着能用、其实不能用的:`/Laser_map`

`/Laser_map` 名字像"累积地图",实际是**初始化阶段 kdtree 建成时发一次**的快照
**[读码]**:`laserMapping.cpp:970` 调用 `publish_init_kdtree()`,函数体里
`flatten(ikdtree.Root_Node, ...)` 后 `publish` 一次;该函数**全文件只有这一个调用点**,
不在主循环里。所以它既不是持续更新的、也不是全量地图 → **不能当先验图用**。

### 2.3 补齐资产(推荐做法)

```bash
# ① 先验点云:改 config/mid360.yaml 的 pcd_save.pcd_save_en: true
#    下次建图 Ctrl+C 退出时写: src/point_lio_ros2/PCD/scans.pcd
#    (ROOT_DIR 是编译期烧进二进制的【源码】目录,不是 install/)
#    注:全分辨率原始点云要重新走一次建图才拿得到,这是唯一"必须重做"的一步。

# ② 2D 占据栅格:直接由运行中的 octomap_server 导出
sudo apt install -y ros-jazzy-nav2-map-server
ros2 run nav2_map_server map_saver_cli -f ~/maps/map2d -t /projected_map
# 注:nav2_map_server 的 CLI 用 `-t <话题>` 选输入地图(默认 "map"),
# 不是 `-r map:=...`;`--ros-args` 若要写必须放在最后(jazzy 分支 usage 原文)
# 得到 ~/maps/map2d.pgm + map2d.yaml(resolution = octomap 的 0.2,
# 高度带由 config/octomap_mid360.yaml 的 occupancy_min_z/max_z = -2.0/3.0 决定)
```

验收:**[实测]** rviz/pcl_viewer 里点云与房间轮廓肉眼一致;pgm 里墙线不糊、无整片空洞。

---

## 3. 架构:盲启动的三段式

```
① 候选生成(G1)   2D 占据栅格 + 一帧扫描 ──BBS 分支定界──▶ top-N 候选 (x, y, yaw)
② 精配准/校验(G2) 候选 + 当前扫描 + map.pcd ──NDT/GICP──▶ 分数(fitness),取最优
③ 给初值/重播种(G3) 最优候选 ──▶ /initialpose(或直接发 TF)
```

TF 关系(与本工作区现有帧名对应,**这是最容易错的地方**):

```
map ──(定位节点发布)──▶ camera_init ──(point_lio 发布)──▶ body
```

- `camera_init` 是 **point_lio 每次启动那一刻**的世界系 **[读码]**,开机位置一变它就变,
  所以"定位"实质就是求 `map → camera_init` 这个变换;
- 里程计照常高频输出(`/aft_mapped_to_init`, `nav_msgs/Odometry` **[实测]**),
  全局坐标只是在拿到 `map→camera_init` 之前"未知",不影响 LIO 自身收敛。
  → 所以流程可以是:**先让 LIO 跑起来 → 后台做全局搜索(几秒级)→ 一次性把变换发出去**。

**point_lio 自己不会做这件事** **[读码]**:源码里没有任何先验地图加载逻辑
(grep `prior` / `load pcd` / `map_file` 全无命中;`laserMapping.cpp` 里只有 `Localmap_Initialized`);
`launch/correct_odom_*.launch.py` 唯一的作用是把 `odom_only: true` 打开(只发里程计),**不是定位**。
→ 定位能力**必须外挂**(这就是 §4 那堆第三方包存在的理由)。

---

## 4. 方案对比(逐个查证)

| 方案 | 上游状态 | ROS2/Jazzy | 盲启动能力 | 结论 |
|---|---|---|---|---|
| **`rsasaki0109/lidar_localization_ros2`** | v1.2.0,活跃(持续在 PR/benchmark) | **[上游]** "ROS 2 Jazzy with NDT_OMP is the recommended starting point";`main` 分支即 Jazzy | ✅ 有 opt-in 的 G1/G2/G3 三段(`global_localization_node.py` 暴露 `~/query` service;BBS_2D over 占据栅格;可选 NDT 精配准;`reinitialization_supervisor_node.py` 做 guarded 重播种) | **3D 路线首选**,但见下面的诚实边界 |
| `koide3/hdl_global_localization` | BBS + FPFH/RANSAC 的参考实现(rsasaki 的 roadmap 明确点名参考它,BSD-2) | ❌ 只有 `master` 分支 = ROS1,最后提交 2024-09 **[实测]** | ✅ 本身就是全局定位包 | 要么自己移植,要么只借它的算法/参数思路 |
| `KOKIAOKI/3d_bbs` | ICRA2024 3D-BBS,精度/速度最好 | ROS2 示例存在(`ros2_test/`) | ✅ | ❌ **本机无 NVIDIA GPU,上游注明 ROS2 示例仅支持 GPU [未验证]** → 排除 |
| `scancontext_ros2`(aserbremen / ChiratheRobotics fork) | 12 star,**最后提交 2023-09 [实测]** | ✅ 是 ROS2 包 | ⚠️ 只做**描述子检索**(给地点 + yaw 候选),之后仍需 ICP 精配准 | 低维护,自己接的量大;有走廊/重复结构歧义时不如 BBS 稳定 |
| `hdl_localization`(koide3) | 老牌 NDT+UKF | ❌ 官方**无 ROS2 分支**(分支仅 devel/master/global_localization/gpu… **[实测]**) | ❌ 它解决的是"**有初步粗值**之后的跟踪",不是盲启动 | ROS2 得用第三方移植 `Taeyoung96/hdl-localization-ROS2`(Humble 时代),工作量大 |
| `POINT_LIO_LOCALIZATION` | 基于 point_lio 的 ICP 重定位框架 | ROS1 官方 + 一个 Humble 移植 | ⚠️ 需要粗初值 + ICP 精配准 | ❌ 本工作区已对 point_lio 打 P1/P2/P4/P5 补丁,**换 fork 要重新对齐**成本最高(违反 `PATCHES.md` 约定 B) |
| **`pointcloud_to_laserscan` + `nav2_amcl`** | Nav2 官方 | ✅ apt:1.3.12 / 2.0.2 **[实测]** | ✅ AMCL 就是**多假设粒子滤波**,天然"不知道在哪"起步 | **2D 路线首选**,最省事;3D 姿态/非平面场景不适用 |

### 4.1 上游自己声明的边界(重要,别被"有 relocalization"误导)

`lidar_localization_ros2` 的 `docs/v1_status.md` 里 **[上游原文]**:

- "The current relocalization work is **not production-grade global relocalization**."
- "Occupancy-map generation from PCD is **route-crop oriented and is not a general mapper**."
- "Do not claim: production-ready global relocalization."
- 时间/覆盖:单次 query 在验证过的窗口上"几秒级";候选精度上,未经 NDT 精配准的 BBS 原始候选
  "**up to ~3 m off**"(上游在 HDL 数据集上实测)。
- 它的 G1 是**离线/按需**生成候选("artifact-first"),G3 是**guarded** 才发重置 —— 
  即默认路径上 `published_count=0`,要显式打开。

结论:它不是"黑盒解决盲启动",而是"把候选生成→精配准→校验这条流水线给你拼好了"。

---

## 5. 推荐路线(分三阶段,每阶段带验收标准)

### P0 补齐先验资产(必做,不做后面全是空转)

见 §2.3。验收:`scans.pcd` 存在且能在 rviz 里显示;`map2d.pgm` 的墙线与实测一致。

### P1 先用 2D AMCL 打通"盲启动"闭环(最便宜的可用解)

```bash
# 终端 1:LIO(定位阶段不需要 octomap,省 CPU)
ros2 launch mid360_bringup mid360.launch.py show_rviz:=false

# 终端 2:点云转 LaserScan(喂机体系 deskew 过的点云)
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -r cloud_in:=/cloud_registered_body -r scan:=/scan \
  -p target_frame:=body -p min_height:=-0.5 -p max_height:=0.5

# 终端 3:发布先验 2D 栅格(map_server 是【生命周期节点】,要手工切状态)
sudo apt install -y ros-jazzy-nav2-map-server
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=$HOME/maps/map2d.yaml
ros2 lifecycle set /map_server configure && ros2 lifecycle set /map_server activate

# 终端 4:AMCL —— odom_frame_id 用本工作区的 camera_init,让 AMCL 独占 map→camera_init 这条边
ros2 run nav2_amcl amcl --ros-args \
  -p base_frame_id:=body -p odom_frame_id:=camera_init -p global_frame_id:=map \
  -p scan_topic:=/scan
```

验收:AMCL 粒子云从全图摊开 → 收敛到一处且位置正确;`map→camera_init` 变换稳定;
`ros2 run tf2_tools view_frames` 里每条边只有一个分发者。

> 若建图那套还开着、`/projected_map` 正在发,也可以让 AMCL 直接吃它(省掉 map_server),
> 但注意 `octomap_mid360.yaml` 里是 `latch: false`(非瞬态),后加入的 AMCL 要等下一次地图更新才收得到。

### P2 需要 3D 时再上 NDT/BBS 路线

```bash
sudo apt install -y python3-vcstool
mkdir -p ~/lidarloc_ws/src && cd ~/lidarloc_ws/src
git clone https://github.com/rsasaki0109/lidar_localization_ros2.git
cd lidar_localization_ros2
vcs import --input dependencies.repos .      # 拉 ndt_omp_ros2(注意版本锁的是 humble 分支)
cd ~/lidarloc_ws && colcon build             # 或 scripts/bootstrap_colcon_workspace.sh --build
```

上游的 MID-360 现成 launch(参数可直接覆盖成本工作区的帧/话题):

```bash
ros2 launch lidar_localization_ros2 mid360_legged_localization.launch.py \
  map_path:=$HOME/maps/scans.pcd \
  cloud_topic:=/cloud_registered_body \
  imu_topic:=/livox/imu \
  global_frame_id:=map odom_frame_id:=camera_init base_frame_id:=body \
  registration_method:=NDT_OMP ndt_num_threads:=2   # 本机 4 核,别用默认 4
```

盲启动那一段(G1→G2→G3,全 opt-in):

```bash
# G1 离线:从 PCD 生成占据栅格 + 生成候选
python3 scripts/generate_occupancy_map_from_pcd.py ...        # 生成 occupancy.yaml
python3 scripts/make_bbs_relocalization_attempts.py ...       # 离线候选(artifact)

# G2 运行期服务:按需查询,返回排序候选(JSON)+ PoseArray
python3 scripts/global_localization_node.py --ros-args \
  -p occupancy_yaml:=/path/to/occupancy.yaml \
  -p cloud_topic:=/cloud_registered_body \
  -p global_frame_id:=map
ros2 service call /global_localization_node/query std_srvs/srv/Trigger
ros2 topic echo /global_localization_node/status      # 慢了看进度,别黑等

# G3 监督重播种(有安全门控,默认不发布)
python3 scripts/reinitialization_supervisor_node.py --ros-args \
  -p query_service:=/global_localization_node/query \
  -p alignment_status_topic:=/alignment_status \
  -p initialpose_topic:=/initialpose
```

验收:**同一位置重启/搬动 3 次,top-1 候选都给对(误差 < 0.5 m)**;
走廊等重复结构上看候选分歧度(见 §6.3)。

---

## 6. 坑(逐条)

### 6.1 `camera_init` 每次启动都是新的世界系
**定位 = 求 `map→camera_init`**。TF 只能有**一个**分发者:
`point_lio` 已经发 `camera_init→body`,定位节点就只发 `map→camera_init`;
别再让第二个节点发同一条边(重复发布会导致 TF 抖动/跳变)。

### 6.2 全局定位锁上的一瞬间会"瞬跳"
`body` 的全局位姿可能一次跳好几米。下游(costmap / 导航 / 控制)必须支持 reset,
否则会把跳变当障碍物或直接把速度指令拉爆。**保底手段:让 top 候选先显示在 rviz 里人工确认**
(这也正是上游把 G3 做成"guarded、默认不发布"的原因)。

### 6.3 几何歧义是原理性的,不是调参能解的
长走廊、重复房间、空旷大厅 → 2D BBS / Scan Context 都会给出**多个同分候选**。
必须:top-N 候选 + NDT fitness 校验 + (必要时)让机器人动一段用运动约束消歧。
**任何"全自动盲启动 100% 成功"的承诺在室内重复结构下都不成立。**

### 6.4 精度天花板由先验资产决定
先验如果来自 `.bt`(0.2 m 体素中心)→ 定位精度与 fitness 都比全分辨率 PCD 差一档。
要定位精度就别省 §2.3 的第 ① 步。

### 6.5 CPU 预算(本机 4 核)
`point_lio`(10 Hz)+ `octomap_server` + NDT_OMP 抢同一台机器:
- 定位阶段**关掉 octomap**(定位链路完全不需要它,它只服务 2D 投影/避障);
- `ndt_num_threads` 用 2,别用默认 4;
- 参考:`resolution: 0.1` 时 octomap_server 曾占 **78.6% CPU / 每帧 82 ms**
  (`config/octomap_mid360.yaml` 里记着这笔账)。

### 6.6 候选"看着对但锁不上":先看时间
用"最近一帧点云"做匹配 → 上游文档明确说 `candidate_age_sec` 是排查这种情况的**第一个量**。
机器人是移动的、查询又慢时,候选对应的扫描早就过期了。

### 6.7 帧名对齐要用参数覆盖,别改 vendored 代码
上游默认 `map/odom/base_link`,本工作区实际是 `camera_init/body`(`PATCHES.md` P2)。
用 `global_frame_id/odom_frame_id/base_frame_id` 覆盖 —— `PATCHES.md` 约定 B:
vendored 代码只允许最小就地补丁。

### 6.8 `/cloud_registered_body` 是 deskew 过的(P4)
喂定位的应该是这份机体系点云;喂世界系 `/cloud_registered` 会让"射线/初始位姿"语义错
(同样的坑在 octomap 那边已经踩过一次,见 `octomap_mid360.yaml` 顶部注释)。

---

## 7. 待实测清单(本工作区尚未验证)

| # | 项 | 怎么验 | 通过标准 |
|---|---|---|---|
| 1 | `ndt_omp_ros2` 的 **`humble` 分支**能否在 Jazzy/24.04 编译 | `vcs import` + `colcon build --packages-up-to lidar_localization_ros2` | 编译通过;不行就把 `dependencies.repos` 里的 `humble` 改成 `main`,或自己打 CMake 补丁 |
| 2 | `lidar_localization_ros2` 生命周期节点能否激活 | 上游 `check_lidar_localization_bringup.py --profile standalone` | 生命周期到 active,`/pcl_pose` 有输出 |
| 3 | MID-360 launch 吃 `/cloud_registered_body` | 覆盖 `cloud_topic` 后看 `alignment_status` | fitness 正常收敛,不报 "no points" |
| 4 | `/projected_map` 导出的 2D 栅格质量 | 开 rviz 叠 `map2d.pgm` 与实际比对 | 墙线不糊、门洞保留、无整片空洞 |
| 5 | AMCL 路线的 TF 衔接 | `ros2 run tf2_tools view_frames` | 全链唯一:`map→camera_init→body`,无重复发布者 |
| 6 | `scans.pcd` 体积/内存 | `ls -lh src/point_lio_ros2/PCD/` | 能一次性载入(本机空闲约 11 GiB);长测改 `pcd_save.interval` |
| 7 | 盲启动成功率(验收基线) | 同一先验下,搬动/重启 3 次做 top-1 命中统计 | ≥ 2/3 命中且误差 < 0.5 m;不达标先查 §6.3 歧义 |

---

## 8. 参考

**上游 / 第三方**

- `rsasaki0109/lidar_localization_ros2` — https://github.com/rsasaki0109/lidar_localization_ros2
  (重点看 `docs/global_localization.md` G1/G2/G3、`docs/v1_status.md` 的能力边界、
  `docs/global_localization_roadmap.md` 的算法参考)
- `koide3/hdl_global_localization` — https://github.com/koide3/hdl_global_localization (BBS / FPFH+RANSAC,ROS1)
- `KOKIAOKI/3d_bbs` — https://github.com/KOKIAOKI/3d_bbs (ICRA2024,GPU)
- `scancontext_ros2` — https://github.com/aserbremen/scancontext_ros2 (描述子检索,低维护)
- `Taeyoung96/hdl-localization-ROS2` — https://github.com/Taeyoung96/hdl-localization-ROS2 (Humble 时代的 hdl_localization)
- `U-AMC/POINT_LIO_LOCALIZATION` / `vanttec/POINT_LIO_LOCALIZATION_ROS2_HUMBLE` (基于 point_lio 的 ICP 重定位)

**本工作区相关文件**

| 文件 | 与本文的关系 |
|---|---|
| `src/mid360_bringup/config/octomap_mid360.yaml` | `resolution` / `max_range` / `occupancy_min_z/max_z`(决定 2D 栅格高度带) |
| `src/point_lio_ros2/config/mid360.yaml` | `pcd_save.pcd_save_en`(P0 第一步)、`odom_child_frame_id: body` |
| `docs/PATCHES.md` | D 项(落盘数据未做)、B 项(vendored 只允许最小补丁)、P2(帧名)、P4(body 云 deskew) |
| `README.md` | 输出话题表(`/octomap_*`、`/projected_map`)、存图命令 |
