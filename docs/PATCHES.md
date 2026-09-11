# 本地补丁与约定清单

本仓库把两个上游包（各自独立仓库）扁平化进了 `src/`，因此**必然存在对上游代码的本地改动**。
这份清单是那些改动的唯一索引：每条都在源码里带 `LOCAL PATCH Pn` 注释，两边一一对应。

**为什么要维护它**：`point_lio_ros2` 是 vendored 代码（不属于本项目）。它最危险的失败模式不是崩溃，
而是**补丁被上游同步覆盖后静默失效** —— 节点照常启动、IMU 初始化到 100%、rviz 里永远没有点云
（详见 commit `4ec2c6a` 的现场记录）。没有清单，这种回归只能靠翻 git 历史发现。

---

## 1. 上游基线

| 包 | 上游 | 基线 | 备注 |
|---|---|---|---|
| `point_lio_ros2` | `github.com/dfloreaa/point_lio_ros2`（ROS2 移植版） | `a8e2d0d`（2025-08-14） | 本地 `~/Downloads/point_lio_ros2_upstream` |
| `point_lio_ros2`（对照参考） | `github.com/hku-mars/Point-LIO`（ROS1 官方） | 本地快照 | 本地 `~/Downloads/Point-LIO_ros1_official`，写补丁时以它为准 |
| `livox_ros_driver2` | Livox 官方 | 未固定 commit（CHANGELOG 顶端 `1.2.7`） | 导入时未记录 commit，建议以后补上 |
| `mid360_bringup` | — | — | **本项目自有代码，不是 fork**，不受本清单约束 |

## 2. 补丁清单

| 编号 | 文件 | 改动 | 为什么必须这么做 | 上游/官方对照 |
|---|---|---|---|---|
| **P1** | `src/point_lio_ros2/src/preprocess.h`<br>`src/point_lio_ros2/src/preprocess.cpp` | 1) 新增 `livox_ros::Point` PCL 点结构（`timestamp` 为 float64 绝对 ns）<br>2) 新增 `Preprocess::livox_handler()`<br>3) `switch(lidar_type)` 新增 `case AVIA:` 调用它 | 上游 ROS2 移植版**没有 livox 分支**（只有 OUST64/VELO16/HESAIxt32/UNILIDAR，`avia_handler` 整段被注释）。缺了它，MID-360 的 PointCloud2 会落到 `default: printf("Error LiDAR Type")`，`pl_surf` 为空 → 静默 0 点云 | ROS1 官方 `avia_handler`（`preprocess.cpp:316-362`）用 CustomMsg 的 `offset_time`；本补丁把等价信息从 PointCloud2 的 `timestamp` 取出来：`curvature = (t_i - t_0) * 1e-6` ms。时间基准与官方等价的证据：`GetLidarBaseTime()` = 首点时间（`pub_handler.cpp:281-286`）→ `header.stamp`（`lddc.cpp:310/316`） |
| **P2** | `src/point_lio_ros2/config/mid360.yaml` | 1) 设 `odom_child_frame_id: "body"`<br>2) `scan_bodyframe_pub_en: true`<br>3) 显式写出降采样参数：`filter_size_surf: 0.25`（代码默认 0.5）、`filter_size_map: 0.5` | ① 代码里 `/cloud_registered_body` 的 `frame_id` 是**硬编码** `"body"`（`laserMapping.cpp:593`），同文件其余帧都走 `odom_*_frame_id` 参数；配置不改就出现 "Frame camera_init does not exist"/TF 查不到。② octomap_server 把 `lookupTransform(frame_id, 点云 frame)` 的平移当射线起点，喂世界系点云会让所有射线从世界原点射出。③ `filter_size_surf` 既决定逐点 EKF 的输入点数，也决定 `/cloud_registered`（世界系点云）的密度——默认 0.5 m 时只有 ~900 点/帧，rviz 里看着“稀/卡”；0.25 → 约 2~4 倍，代价是 CPU 上升一档 | 上游配置文件只有默认值；三处都是按本项目的 octomap / 显示链路要求改的 |
| **P3** | `src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py` | `xfer_format` 默认值 `1` → `0` | `point_lio` 订阅 `sensor_msgs/PointCloud2`,而 `xfer_format=1` 发的是 `livox_ros_driver2/CustomMsg`;默认值为 1 时用户不加参数就静默收不到点云 | 官方默认 1;改回 0 与 P1 配套。要发 CustomMsg 需显式传参**且**先恢复 CustomMsg 通路（见下） |
| **P4** | `src/point_lio_ros2/src/laserMapping.cpp`<br>`publish_frame_body()` | 发布 `/cloud_registered_body` 前用估计运动做逐点 deskew：`p_ref = Exp(−Δt·ω)·p − Δt·v_body`（Δt = lidar_end_time − 点时刻；ω 取 gyro−bg 或 `x_.omg`；`x_.vel` 是世界系，先用 Rᵀ 转回机体系） | **官方在这一环不做补偿**（ROS1 `IMU_Processing.cpp:102` 只做 `*cur_pcl_un_ = *(meas.lidar)`，ROS1/ROS2 的 `publish_frame_body` 都只套固定外参、stamp 用 `lidar_end_time`）。于是 octomap_server 按单一时戳查一次 TF，把所有点当成同一瞬间测得的：旋转时每点错位 ω·(t_ref−t_i)，一帧最坏 ω×100 ms（90°/s ≈ 9°，3 m 处约 47 cm）= **旋转重影的结构性来源**。世界系点云不缺这一环（逐点循环里已按各点时刻 `pointBodyToWorld`） | 无上游对照——官方即为此行为。开关：文件内 `static const bool kDeskewBodyCloud`，置 false 可复现官方行为做 A/B。数值验证：常 twist 仿真下逐点误差 0.00 mm（未补偿 80.9 cm） |
| **P5** | `src/point_lio_ros2/launch/mapping_mid360.launch.py` | 从参数字典里删除 `'filter_size_surf': 0.5`、`'filter_size_map': 0.5` | 节点 `parameters` 列表里该字典排在 `params_file` **之后**,会覆盖 yaml —— 于是「在 `config/mid360.yaml` 里改这两项」静默失效(实测踩过:节点实际仍是 0.5,点云仍然 ~900 点/帧)。删除后 yaml 成为唯一真相源(约定 A)。⚠️ 该字典里**仍**盖住 yaml 的还有：`use_imu_as_input=False`、`prop_at_freq_of_imu`、`check_satu`、`init_map_size`、`point_filter_num=3`、`space_down_sample`、`cube_side_length`、`runtime_pos_log_enable`——改这些必须改 launch | 上游原样硬编码。注意 `use_imu_as_input: False` 是算法模式开关（代码默认 true），切错会换整个动力学模型 |
| **P6** | `src/point_lio_ros2/config/mid360.yaml` | `pcd_save.pcd_save_en: false → true`（`interval` 保持 `-1`） | **先验点云是重定位的前置资产**（`docs/LOCALIZATION.md` 的 P0 第一步）。三点必知：① 只有节点**正常退出**（Ctrl+C）才写一次，写盘代码在 `laserMapping.cpp:1358`（主循环之后），`kill -9`/断电不写；② 写到 `src/point_lio_ros2/PCD/scans.pcd` —— `ROOT_DIR` 是编译期烧进二进制的**源码**目录（`CMakeLists.txt:13`），不是 `install/`；③ 实测增长 ≈ 0.5 MB/s 内存、0.25 MB/s 文件，本机 11 GiB 空闲下 `-1` 约 40 min 内安全，超了改 `interval: 6000`（代价：产出多个 `scans_<i>.pcd`，喂 NDT 前要先合并） | 上游默认 `false`（即本清单 D 项原写的“未做”状态） |
| **P7** | `src/point_lio_ros2/launch/mapping_mid360.launch.py`<br>`src/mid360_bringup/launch/mid360.launch.py` | 新增 launch 参数 `pcd_save`（默认 `''`），用 `OpaqueFunction` **条件**追加 `{'pcd_save': {'pcd_save_en': bool}}`；一键 launch 负责透传 | 目标是让“是否落盘 PCD”能**启动时开关**，而不必改 yaml + `colcon build`（`install/` 是拷贝，不 build 不生效）。必须是**条件**追加：节点 `parameters` 列表后者覆盖前者，无条件追加 = 把 yaml 顶掉（就是 P5/约定 A 那个坑）。注：运行期 `ros2 param set /laserMapping pcd_save.pcd_save_en true` **无效** —— `parameters.cpp:154` 只在启动时 `get_parameter` 一次进普通 bool，全文件无参数回调 | 上游无此参数；上游默认 `pcd_save_en: false` |
| **S1** | 仓库结构 | 两个上游包扁平化进 `src/`；移除失效的 `.gitmodules` / 子模块（`3rdparty/IKFoM`、`ikd-Tree` 直接入库） | 便于单工作空间一次 `colcon build`，避免嵌套仓库的 submodule 同步问题 | commit `8635063`、`8938f25` |

### 尚未恢复的上游通路（**故意**保持注释）

`Preprocess::process(CustomMsg)`、`avia_handler`、`laserMapping.cpp` 的 `livox_pcl_cbk` 与 CustomMsg 订阅，
以及 `package.xml`/`CMakeLists.txt` 里对 `livox_ros_driver2` 的依赖，上游均为注释状态。

要用 CustomMsg（`xfer_format:=1`）时必须先全部恢复，**但不要照抄 `preprocess.cpp` 里那段注释代码**：
它带一段 `curvature` 单调性 hack（相邻点差 <1 ms 就改成 `前点 + 0.004166667f`）且缺 `det_range` 判断；
应以 ROS1 官方 `avia_handler` 为准。相关分析见下节 B 项。

## 3. 项目自身的约定（非上游补丁，但同样容易被改回去）

| 编号 | 约定 | 现状 | 理由 |
|---|---|---|---|
| **A** | `resolution` / `max_range` 等节点参数的**唯一真相源是 `config/*.yaml`**；launch 的 `DeclareLaunchArgument` 一律 `default_value=''`，只有显式传参才覆盖 | 已做（`octomap_mid360.launch.py` 用 `OpaqueFunction` 组装参数） | 旧写法在 launch 里给了 `default_value='0.1'`，而节点 `parameters` 列表**后者覆盖前者** → 无条件把 yaml 顶掉，"改了 yaml 不生效"。运行时会读到 `install/` 下的副本，更容易误判 |
| **B** | vendored 代码（`point_lio_ros2`、`livox_ros_driver2`）**只允许最小就地补丁 + `LOCAL PATCH` 标记**，不做架构改造（不引入插件注册表等） | 已立规 | 每次同步上游都要三方合并；架构改造会让同步成本失控。抽象应该放在自己的边界（`mid360_bringup`）上 |
| **C** | 启动前跑 `scripts/preflight.sh` 静态自检，不自洽就拒绝启动 | 已做（`mid360.launch.py` 在 `generate_launch_description` 阶段同步调用） | 本链路最危险的失败是"静默 0 点云"而不是崩溃。该脚本会检查：`xfer_format` 与 `lidar_type` 匹配、P1 补丁是否还在（grep `case AVIA`/`livox_handler`）、`timestamp_unit` 单位、话题名、octomap 是否吃机体系点云、`src` 改了是否忘了 `colcon build`。跳过：`MID360_SKIP_PREFLIGHT=1` |
| **D** | 调参期把数据落盘（`pcd_save_en` / `runtime_pos_log_enable`），便于事后复现 | **部分做**：`pcd_save_en: true`（P6），并可由 launch 参数 `pcd_save:=true|false` 临时开关（P7）；`runtime_pos_log_enable` 仍为 false —— 且它被 `mapping_mid360.launch.py` 的参数字典**盖住**（见 P5），要改必须改 launch | `Log/*.txt` 每次启动清零，重影/漂移这类问题没有原始数据就只能靠现场盯屏 |

## 4. 同步上游时的流程

```bash
cd ~/Projects/ros_ws

# 1) 差异应该只剩本清单列出的那些文件(P1/P2 + 上游自带的 .gitmodules)
diff -rq src/point_lio_ros2 ~/Downloads/point_lio_ros2_upstream

# 2) 就地标记是否都还在(每处补丁都带 LOCAL PATCH Pn)
grep -rn "LOCAL PATCH" src/ docs/

# 3) 补丁是否仍然**生效**(会实际 grep case AVIA / livox_handler 并检查配置自洽)
./src/mid360_bringup/scripts/preflight.sh

# 4) 确认无误后重建(install/ 是拷贝,不是软链,不 build 不生效)
colcon build
```

第 1 步的**期望输出**（多出来的文件说明上游改过或补丁丢了,需要人工判断）：

```
文件 src/point_lio_ros2/config/mid360.yaml 和 .../config/mid360.yaml 不同     # P2
文件 src/point_lio_ros2/src/preprocess.cpp 和 .../src/preprocess.cpp 不同     # P1
文件 src/point_lio_ros2/src/preprocess.h 和 .../src/preprocess.h 不同         # P1
文件 src/point_lio_ros2/src/laserMapping.cpp 和 .../src/laserMapping.cpp 不同 # P4
只在 ~/Downloads/point_lio_ros2_upstream 中存在：.gitmodules                   # S1 移除
```

## 5. 相关文档

- `README.md` —— 工作空间总览与一键启动
- `docs/PATCHES.md`（本文）—— 上游补丁索引 + A/B/C/D 约定
- `src/mid360_bringup/launch/*.launch.py` 文件头注释 —— 这一层的设计取舍
  （为什么用 `show_rviz` 而不是 `rviz`、为什么 `default_value=''`）
- `src/mid360_bringup/config/octomap_mid360.yaml` 顶部注释 —— 为什么必须喂机体系点云、`occupancy_min_z/max_z` 只影响 2D 投影
