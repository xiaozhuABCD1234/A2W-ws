# 本地补丁与约定清单

本仓库把两个上游包（各自独立仓库）扁平化进了 `src/`：`mid360_driver` 与
`point_lio_ros2`。因此**必然存在对上游代码的本地改动**。这份清单是那些改动的唯一索引：
每条都在源码里带 `LOCAL PATCH Pn` 注释，两边一一对应。

**为什么要维护它**：`point_lio_ros2` 是 vendored 代码（不属于本项目）。它最危险的失败模式
不是崩溃，而是**补丁被上游同步覆盖后静默失效** —— 节点照常启动、IMU 初始化到 100%、
rviz 里永远没有点云（历史上的 livox `xfer_format` 错配就是这样，详见 commit `4ec2c6a`
的现场记录）。没有清单，这种回归只能靠翻 git 历史发现。

---

## 1. 上游基线

| 包 | 上游 | 基线 | 备注 |
|---|---|---|---|
| `point_lio_ros2` | `github.com/dfloreaa/point_lio_ros2`（ROS2 移植版） | `a8e2d0d`（2025-08-14） | 本地 `~/Downloads/point_lio_ros2_upstream` |
| `point_lio_ros2`（对照参考） | `github.com/hku-mars/Point-LIO`（ROS1 官方） | 本地快照 | 本地 `~/Downloads/Point-LIO_ros1_official`，写补丁时以它为准 |
| `mid360_driver` | `github.com/Yancey2023/mid360_driver`（upstream/main `e2ba0ca`） | 本地 fork `xiaozhuABCD1234/mid360_driver` 的 `4adf8df`（`add mid70 driver`，含 Mid-70 支持） | MIT，作者 Yingjie Huang；本地 `~/Projects/mid360_driver` |
| `mid360_bringup` | — | — | **本项目自有代码，不是 fork**，不受本清单约束 |
| `point_lio_backend` | — | — | **本项目自有代码**（回环检测 + Ceres 位姿图 + map 系纠偏），不修改任何 vendored 包，不受本清单约束 |

## 2. 补丁清单

| 编号 | 文件 | 改动 | 为什么必须这么做 | 上游/官方对照 |
|---|---|---|---|---|
| **P1** | `src/point_lio_ros2/src/preprocess.h`<br>`src/point_lio_ros2/src/preprocess.cpp` | 1) 新增 `Preprocess::livox_handler()`：用 `sensor_msgs::PointCloud2ConstIterator` **按字段名**取 `x/y/z/intensity`（float32）与 `timestamp`（float64），不再依赖 PCL 点结构体布局；删除旧 `livox_ros::Point` 结构体注册；新布局没有 `tag/line`<br>2) `switch(lidar_type)` 新增 `case AVIA:` 调用它<br>3) 帧内时间 = `timestamp - header.stamp`（`timestamp` 为绝对 Unix **秒**），按 `timestamp_unit=0` 换算成 ms；字段缺失会打印 `invalid PointCloud2 layout` 并返回空点云（不崩溃） | 上游 ROS2 移植版**没有 livox 分支**（只有 OUST64/VELO16/HESAIxt32/UNILIDAR，`avia_handler` 整段被注释）。缺了它，MID-360 的 PointCloud2 会落到 `default: printf("Error LiDAR Type")`，`pl_surf` 为空 → 静默 0 点云 | ROS1 官方 `avia_handler`（`preprocess.cpp:316-362`）用 CustomMsg 的 `offset_time`；新驱动把每点绝对时间直接放在 `timestamp` 里，本补丁以 `header.stamp`（帧内最早点）为基准，`curvature = (t_i - header.stamp) * 1e3` ms。旧布局的 `tag/line` 过滤已随驱动废弃 |
| **P2** | `src/point_lio_ros2/config/mid360.yaml` | 1) 设 `odom_child_frame_id: "body"`<br>2) `scan_bodyframe_pub_en: true`<br>3) 显式写出降采样参数：`filter_size_surf: 0.25`（代码默认 0.5）、`filter_size_map: 0.5` | ① 代码里 `/cloud_registered_body` 的 `frame_id` 是**硬编码** `"body"`（`laserMapping.cpp:593`），同文件其余帧都走 `odom_*_frame_id` 参数；配置不改就出现 "Frame camera_init does not exist"/TF 查不到。② octomap_server 把 `lookupTransform(frame_id, 点云 frame)` 的平移当射线起点，喂世界系点云会让所有射线从世界原点射出。③ `filter_size_surf` 既决定逐点 EKF 的输入点数，也决定 `/cloud_registered`（世界系点云）的密度——默认 0.5 m 时只有 ~900 点/帧，rviz 里看着“稀/卡”；0.25 → 约 2~4 倍，代价是 CPU 上升一档 | 上游配置文件只有默认值；三处都是按本项目的 octomap / 显示链路要求改的 |
| **P4** | `src/point_lio_ros2/src/laserMapping.cpp`<br>`publish_frame_body()` | 发布 `/cloud_registered_body` 前用估计运动做逐点 deskew：`p_ref = Exp(−Δt·ω)·p − Δt·v_body`（Δt = lidar_end_time − 点时刻；ω 取 gyro−bg 或 `x_.omg`；`x_.vel` 是世界系，先用 Rᵀ 转回机体系） | **官方在这一环不做补偿**（ROS1 `IMU_Processing.cpp:102` 只做 `*cur_pcl_un_ = *(meas.lidar)`，ROS1/ROS2 的 `publish_frame_body` 都只套固定外参、stamp 用 `lidar_end_time`）。于是 octomap_server 按单一时戳查一次 TF，把所有点当成同一瞬间测得的：旋转时每点错位 ω·(t_ref−t_i)，一帧最坏 ω×100 ms（90°/s ≈ 9°，3 m 处约 47 cm）= **旋转重影的结构性来源**。世界系点云不缺这一环（逐点循环里已按各点时刻 `pointBodyToWorld`） | 无上游对照——官方即为此行为。开关：文件内 `static const bool kDeskewBodyCloud`，置 false 可复现官方行为做 A/B。数值验证：常 twist 仿真下逐点误差 0.00 mm（未补偿 80.9 cm） |
| **P5** | `src/point_lio_ros2/launch/mapping_mid360.launch.py` | 从参数字典里删除 `'filter_size_surf': 0.5`、`'filter_size_map': 0.5` | 节点 `parameters` 列表里该字典排在 `params_file` **之后**,会覆盖 yaml —— 于是「在 `config/mid360.yaml` 里改这两项」静默失效(实测踩过:节点实际仍是 0.5,点云仍然 ~900 点/帧)。删除后 yaml 成为唯一真相源(约定 A)。⚠️ 该字典里**仍**盖住 yaml 的还有：`use_imu_as_input=False`、`prop_at_freq_of_imu`、`check_satu`、`init_map_size`、`point_filter_num=3`、`space_down_sample`、`cube_side_length`、`runtime_pos_log_enable`——改这些必须改 launch | 上游原样硬编码。注意 `use_imu_as_input: False` 是算法模式开关（代码默认 true），切错会换整个动力学模型 |
| **P6** | `src/point_lio_ros2/config/mid360.yaml` | `pcd_save.pcd_save_en: false → true`（`interval` 保持 `-1`） | **先验点云是重定位的前置资产**（`docs/LOCALIZATION.md` 的 P0 第一步）。三点必知：① 只有节点**正常退出**（Ctrl+C）才写一次，写盘代码在 `laserMapping.cpp:1358`（主循环之后），`kill -9`/断电不写；② 写盘目录见 P8（`./PCD/scans.pcd`，相对运行 launch 的目录）；③ 实测增长 ≈ 0.5 MB/s 内存、0.25 MB/s 文件，本机 11 GiB 空闲下 `-1` 约 40 min 内安全，超了改 `interval: 6000`（代价：产出多个 `scans_<i>.pcd`，喂 NDT 前要先合并） | 上游默认 `false`（即本清单 D 项原写的“未做”状态） |
| **P7** | `src/point_lio_ros2/launch/mapping_mid360.launch.py`<br>`src/mid360_bringup/launch/mid360.launch.py` | 新增 launch 参数 `pcd_save`（默认 `''`），用 `OpaqueFunction` **条件**追加 `{'pcd_save': {'pcd_save_en': bool}}`；一键 launch 负责透传 | 目标是让“是否落盘 PCD”能**启动时开关**，而不必改 yaml + `colcon build`（`install/` 是拷贝，不 build 不生效）。必须是**条件**追加：节点 `parameters` 列表后者覆盖前者，无条件追加 = 把 yaml 顶掉（就是 P5/约定 A 那个坑）。注：运行期 `ros2 param set /laserMapping pcd_save.pcd_save_en true` **无效** —— `parameters.cpp:154` 只在启动时 `get_parameter` 一次进普通 bool，全文件无参数回调 | 上游无此参数；上游默认 `pcd_save_en: false` |
| **P8** | `src/point_lio_ros2/src/laserMapping.cpp`<br>（两个落盘点：`:568` 分片、`:1360` 退出时；另有 `main` 里的 `mkdir("PCD")`） | `string(ROOT_DIR) + "PCD/..."` → `"PCD/..."`：改用**相对路径**，即**运行 `ros2 launch` 那个目录**下的 `PCD/`；并在启动时 `mkdir("PCD", 0755)` 一次（已存在 = `EEXIST`，不算错；建不了会打印 `[pcd_save] mkdir PCD 失败(...)`，否则失败是静默的） | 上游把路径编译期烧死成 `ROOT_DIR`（= 构建时的源码绝对路径，`CMakeLists.txt:13`）：工作空间一换目录/换机器就得重新 `colcon build`，产物还落在源码树里。PCL 的 `savePCDFileBinary` 不自建目录（上游靠 `PCD/temp.txt` 占位，相对路径下那个占位文件不起作用），所以补了 mkdir。`pcd_view.launch.py` 的默认文件路径同步改成 `./PCD/scans.pcd` | 上游行为见 `laserMapping.cpp:1360` |
| **P9** | `src/point_lio_ros2/config/mid360.yaml` | `timestamp_unit: 3 → 0`（秒） | `mid360_driver` 的每点 `timestamp` 是**绝对 Unix 秒**；若仍按 3（ns）换算，`curvature` 会小 1000 倍，逐点 deskew 的时间轴整体错位（旋转重影的量级） | 上游/旧驱动的点时间是 ns，所以原配置是 3；换驱动必须同步改 |
| **P10** | `src/point_lio_ros2/config/mid360.yaml` | `mapping.acc_norm: 1.0 → 9.81` | 旧 `livox_ros_driver2` 发的 IMU 加速度是 **g**（`pub_handler.cpp` 直接拷 `imu->acc_x`），所以上游用 1.0；`mid360_driver` 发的是 **m/s²**（发布前乘 9.81）。point_lio 代码里是 `acc * G_m_s2 / acc_norm`，若仍用 1.0，所有加速度会被再放大 9.81 倍 → 重力/速度估计系统性失真（静止场景可能不明显，一动就飘） | 上游默认 1.0（对应 g）；换驱动必须同步改，`preflight.sh` 的 C2 会拦 |
| **P11** | `src/point_lio_ros2/launch/mapping_mid360.launch.py` | `_clean_ld_library_path()`：给 `laserMapping` 节点设 `additional_env`，把 `LD_LIBRARY_PATH` 里的 `/opt/MVS` 路径剔除（其余路径保留） | 本机 `~/.bashrc` 把 `/opt/MVS/lib/64` 放在 `LD_LIBRARY_PATH` 最前，其中的旧 `libusb-1.0.so.0` 缺 `libusb_set_option`；系统 `libpcl_io.so.1.14` 需要它 → `pointlio_mapping` 启动即 `symbol lookup error`，exit 127，rviz 随后报 `Frame [camera_init] does not exist`。只影响 laserMapping，不影响用 MVS 相机的节点 | 上游无此逻辑；MVS SDK 更新到带新 libusb 后可删掉 |
| **M1** | `src/mid360_driver/src/asio_compat.hpp`（新增）<br>`src/mid360_driver/src/` 下各 `#include <asio.hpp>`<br>`src/mid360_driver/package.xml` | 新增 `asio_compat.hpp`：include `<boost/asio.hpp>`，并把 `asio` 命名空间转发到 `boost::asio`（含 `error_code` 别名）；源码里的 `#include <asio.hpp>` 改为 `#include "asio_compat.hpp"`；`package.xml` 的依赖 `asio` 改为 `boost` | 本机没有 `libasio-dev` 且无法 sudo；Boost.Asio 与独立 Asio API 兼容，改头文件即可 | 上游原版需要 `libasio-dev`。换机器若想回上游原版，删掉 M1、装回 `libasio-dev` 即可 |
| **M2** | `src/mid360_driver/config/param.yaml` | `host_ip: 192.168.1.50`（本机接雷达网卡 `enp5s0` 的地址）；`lidar_topic` / `imu_topic` 保持 `/livox/lidar` / `/livox/imu` | 驱动只在本机 `host_ip` 上监听 UDP 56301/56401，被动等雷达推流；换机器/网卡必须同步改，否则收不到数据 | 上游默认也是 `192.168.1.50`（示例值）；真正生效的是本地这份配置 |
| **S1** | 仓库结构 | 上游包扁平化进 `src/`（现为 `mid360_driver` + `point_lio_ros2`）；移除 point_lio 上游遗留的失效 `.gitmodules` / 子模块（`3rdparty/IKFoM`、`ikd-Tree` 直接入库） | 便于单工作空间一次 `colcon build`，避免嵌套仓库的 submodule 同步问题 | commit `8635063`、`8938f25` |

### 历史补丁（已随 livox_ros_driver2 移除）

| 编号 | 文件 | 改动 | 现状 |
|---|---|---|---|
| **P3** | `src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py` | `xfer_format` 默认值 `1` → `0`（PointCloud2） | **已废弃**。`livox_ros_driver2` 已从 `src/` 删除（删除前 main HEAD=`4448841`），`xfer_format` 概念随驱动消失；`mid360_driver` 只发 PointCloud2。旧代码可 `git checkout 4448841 -- src/livox_ros_driver2` 恢复（本机缺 `libaprutil1-dev`，不保证可构建） |

### 已废弃：CustomMsg 通路

旧 `livox_ros_driver2` 的 CustomMsg 通路（`Preprocess::process(CustomMsg)`、
`avia_handler`、`laserMapping.cpp` 的 `livox_pcl_cbk` 与 CustomMsg 订阅）已随驱动一起废弃，
**不再恢复**。`mid360_driver` 只发 PointCloud2，point_lio 侧只保留 P1 这一条 PointCloud2 通路。

历史：P3 曾把 `xfer_format` 从 1 改成 0，避免 point_lio 静默收不到点云；现在驱动本身
不再有这个消息类型，问题从源头消失。point_lio 的 `package.xml` / `CMakeLists.txt` 里
对 `livox_ros_driver2` 的依赖仍是上游注释状态，**不要恢复**（驱动已不在）。

## 3. 项目自身的约定（非上游补丁，但同样容易被改回去）

| 编号 | 约定 | 现状 | 理由 |
|---|---|---|---|
| **A** | `resolution` / `max_range` 等节点参数的**唯一真相源是 `config/*.yaml`**；launch 的 `DeclareLaunchArgument` 一律 `default_value=''`，只有显式传参才覆盖 | 已做（`octomap_mid360.launch.py` 用 `OpaqueFunction` 组装参数） | 旧写法在 launch 里给了 `default_value='0.1'`，而节点 `parameters` 列表**后者覆盖前者** → 无条件把 yaml 顶掉，"改了 yaml 不生效"。运行时会读到 `install/` 下的副本，更容易误判 |
| **B** | vendored 代码（`point_lio_ros2`、`mid360_driver`）**只允许最小就地补丁 + `LOCAL PATCH` 标记**，不做架构改造（不引入插件注册表等） | 已立规 | 每次同步上游都要三方合并；架构改造会让同步成本失控。抽象应该放在自己的边界（`mid360_bringup`）上 |
| **C** | 启动前跑 `scripts/preflight.sh` 静态自检，不自洽就拒绝启动 | 已做（`mid360.launch.py` 在 `generate_launch_description` 阶段同步调用） | 本链路最危险的失败是"静默 0 点云"而不是崩溃。该脚本会检查：驱动 `config/param.yaml` / `launch` 是否存在、`lidar_topic` / `imu_topic` 与 point_lio 是否一致、`host_ip` 是否为本机网卡地址、`lidar_type=1`、`timestamp_unit=0`、P1 是否在位（grep `livox_handler` / `PointCloud2ConstIterator`）、octomap 是否吃机体系点云、`src` 改了是否忘了 `colcon build`。**不再检查 `xfer_format`**（驱动已换）。跳过：`MID360_SKIP_PREFLIGHT=1` |
| **D** | 调参期把数据落盘（`pcd_save_en` / `runtime_pos_log_enable`），便于事后复现 | **部分做**：`pcd_save_en: true`（P6），并可由 launch 参数 `pcd_save` 传 `true` / `false` 临时开关（P7）；`runtime_pos_log_enable` 仍为 false —— 且它被 `mapping_mid360.launch.py` 的参数字典**盖住**（见 P5），要改必须改 launch | `Log/*.txt` 每次启动清零，重影/漂移这类问题没有原始数据就只能靠现场盯屏 |
| **E** | 回环/位姿图等"后端"能力放在**自有包** `point_lio_backend`，只订阅前端话题（`/aft_mapped_to_init`、`/cloud_registered_body`），不向 vendored 包加钩子；后端启用时世界系统一为 `map`，由后端发布 `map→camera_init` | 已做（`backend:=true` 默认） | 保持 `point_lio_ros2` / `mid360_driver` 同步上游零成本（约定 B 的延伸）；`backend:=false` 时完全退回 `camera_init` 系 |

## 4. 同步上游时的流程

```bash
cd ~/Projects/A2W-ws

# 1) point_lio 差异应该只剩本清单列出的那些文件(P1/P2/P4/P5/P7/P8/P9/P10/P11 + 上游自带的 .gitmodules)
diff -rq src/point_lio_ros2 ~/Downloads/point_lio_ros2_upstream

# 2) mid360_driver 与上游本地 clone 对比(若有;本地 clone 见 §1)
diff -rq -x .git src/mid360_driver ~/Projects/mid360_driver

# 3) 就地标记 + 关键补丁是否仍在位
grep -rn "LOCAL PATCH" src/ docs/
grep -n "case AVIA\|livox_handler\|PointCloud2ConstIterator" src/point_lio_ros2/src/preprocess.cpp
grep -n "timestamp_unit" src/point_lio_ros2/config/mid360.yaml    # 必须是 0

# 4) 配置自检(会实际检查上面的项)
./src/mid360_bringup/scripts/preflight.sh

# 5) 确认无误后重建(install/ 是拷贝,不是软链,不 build 不生效)
colcon build
# 注意:point_lio_backend 是自有包,不在上面的 diff 范围;它依赖的接口只有
# /aft_mapped_to_init 与 /cloud_registered_body,同步上游后不需要改它,但仍要重新 build。
```

第 1 步的**期望输出**（多出来的文件说明上游改过或补丁丢了，需要人工判断）：

```
文件 src/point_lio_ros2/config/mid360.yaml 和 .../config/mid360.yaml 不同     # P2/P9/P10
文件 src/point_lio_ros2/src/preprocess.cpp 和 .../src/preprocess.cpp 不同     # P1
文件 src/point_lio_ros2/src/preprocess.h 和 .../src/preprocess.h 不同         # P1
文件 src/point_lio_ros2/src/laserMapping.cpp 和 .../src/laserMapping.cpp 不同 # P4/P8
文件 src/point_lio_ros2/launch/mapping_mid360.launch.py 和 ... 不同           # P5/P7
只在 ~/Downloads/point_lio_ros2_upstream 中存在：.gitmodules                   # S1 移除
```

`~/Downloads/...` 是写补丁时的本机路径；如果已经不在了，按上游 URL 重新 clone 即可。

mid360_driver 的预期差异只有 M1（新增 `asio_compat.hpp` + include / `package.xml` 改动）与
M2（`config/param.yaml` 的 `host_ip`）。出现别的文件说明本地改过或同步时被覆盖。

## 5. 相关文档

- `README.md` —— 工作空间总览与一键启动
- `docs/PATCHES.md`（本文）—— 上游基线 + 补丁/约定索引
- `src/mid360_driver/config/param.yaml` —— `host_ip` / 话题名（驱动侧唯一要按机器改的地方）
- `src/mid360_bringup/launch/*.launch.py` 文件头注释 —— 这一层的设计取舍
  （为什么用 `show_rviz` 而不是 `rviz`、为什么 `default_value=''`）
- `src/mid360_bringup/config/octomap_mid360.yaml` 顶部注释 —— 为什么必须喂机体系点云、`occupancy_min_z/max_z` 只影响 2D 投影
