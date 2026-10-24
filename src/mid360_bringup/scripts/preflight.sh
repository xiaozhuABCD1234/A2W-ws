#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# mid360_bringup preflight —— 启动前的配置自洽性检查
#
# 为什么需要它:这条链路上最危险的不是崩溃,而是**静默的错配**。
#   mid360_driver 直接发 PointCloud2(x/y/z/intensity float32 + timestamp
#   float64 绝对 Unix 秒);point_lio 的 preprocess.lidar_type 决定用哪个
#   handler 去解,timestamp_unit 决定把帧内时差缩放到 ms 的系数。
#   错配时 point_lio 不报错、IMU 初始化能到 100%,但点云时间轴错位或永远
#   收不到点云 —— 现场表现为"rviz 里什么都没有/旋转重影",排查成本极高。
#   同类问题:octomap 必须喂机体系点云、驱动 host_ip 必须等于本机网卡地址。
#   这些都交给本脚本在启动前拦住。
#
# 用法:
#   preflight.sh                      # 静态检查:只读配置/源码,不需要 ROS 在跑
#   preflight.sh --runtime            # 运行时检查:核对实际 topic 类型
#   preflight.sh --runtime --timeout 20 --topic /livox/lidar
#   preflight.sh --quiet              # 只输出 FAIL/WARN(给 launch 用)
#
# 退出码:0 通过(WARN 不影响)/ 1 有 FAIL / 2 环境不完整(找不到被检查的文件)
#
# 环境变量:MID360_WS  工作空间根目录(默认从脚本位置向上搜索)
# ─────────────────────────────────────────────────────────────────────────────
set -uo pipefail

FAIL=0
WARN=0
RUNTIME=0
QUIET=0
TIMEOUT=10
LID_TOPIC="/livox/lidar"

while [ $# -gt 0 ]; do
  case "$1" in
    --runtime) RUNTIME=1 ;;
    --quiet)   QUIET=1 ;;
    --timeout) TIMEOUT="${2:-10}"; shift ;;
    --topic)   LID_TOPIC="${2:-/livox/lidar}"; shift ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "未知参数:$1(用 --help)" >&2; exit 2 ;;
  esac
  shift
done

if [ -t 1 ]; then
  C_R=$'\033[31m'; C_Y=$'\033[33m'; C_G=$'\033[32m'; C_D=$'\033[2m'; C_0=$'\033[0m'
else
  C_R=''; C_Y=''; C_G=''; C_D=''; C_0=''
fi
pass() { [ "$QUIET" = 1 ] || printf '  %sPASS%s  %s\n' "$C_G" "$C_0" "$1"; }
warn() { printf '  %sWARN%s  %s\n' "$C_Y" "$C_0" "$1"; WARN=$((WARN + 1)); }
fail() { printf '  %sFAIL%s  %s\n' "$C_R" "$C_0" "$1"; FAIL=$((FAIL + 1)); }
info() { printf '        %s%s%s\n' "$C_D" "$1" "$C_0"; }
head2() { [ "$QUIET" = 1 ] || printf '\n%s\n' "$1"; }

# ── 定位工作空间 ────────────────────────────────────────────────────────────
WS="${MID360_WS:-}"
if [ -z "$WS" ]; then
  d="$(cd "$(dirname "$0")" && pwd)"
  for _ in 1 2 3 4 5 6; do
    if [ -d "$d/src/point_lio_ros2" ]; then WS="$d"; break; fi
    d="$(dirname "$d")"
  done
fi

# 先 src/ 后 install/ 拷贝:正常情况下两者一致,不一致本身会被本脚本报出来
pick() { for f in "$@"; do [ -f "$f" ] && { printf '%s\n' "$f"; return 0; }; done; return 1; }

PIO_CFG=""; PIO_LAUNCH=""; DRV_CFG=""; DRV_LAUNCH=""; OCM_CFG=""; OCM_LAUNCH=""; PREPROC=""
if [ -n "$WS" ]; then
  PIO_CFG="$(pick "$WS/src/point_lio_ros2/config/mid360.yaml" \
                  "$WS/install/point_lio/share/point_lio/config/mid360.yaml")" || true
  PIO_LAUNCH="$(pick "$WS/src/point_lio_ros2/launch/mapping_mid360.launch.py" \
                     "$WS/install/point_lio/share/point_lio/launch/mapping_mid360.launch.py")" || true
  DRV_CFG="$(pick "$WS/src/mid360_driver/config/param.yaml" \
                  "$WS/install/mid360_driver/share/mid360_driver/config/param.yaml")" || true
  DRV_LAUNCH="$(pick "$WS/src/mid360_driver/launch/mid360_driver.launch.py" \
                     "$WS/install/mid360_driver/share/mid360_driver/launch/mid360_driver.launch.py")" || true
  OCM_CFG="$(pick "$WS/src/mid360_bringup/config/octomap_mid360.yaml" \
                  "$WS/install/mid360_bringup/share/mid360_bringup/config/octomap_mid360.yaml")" || true
  OCM_LAUNCH="$(pick "$WS/src/mid360_bringup/launch/octomap_mid360.launch.py" \
                    "$WS/install/mid360_bringup/share/mid360_bringup/launch/octomap_mid360.launch.py")" || true
  PREPROC="$(pick "$WS/src/point_lio_ros2/src/preprocess.cpp")" || true
fi

if [ "$QUIET" = 1 ]; then
  printf '\n[preflight] 配置自检(WS=%s)\n' "${WS:-未找到}"
fi

if [ -z "$PIO_CFG" ]; then
  printf '%sFAIL%s  找不到 point_lio 的 config/mid360.yaml\n' "$C_R" "$C_0"
  info "设 MID360_WS=<工作空间根目录> 后重试,或先 colcon build"
  exit 2
fi

# yaml_get <file> <key> —— 取 "key: value # 注释" 里的 value
yaml_get() {
  sed -n "s/^[[:space:]]*$2:[[:space:]]*\([^#]*\).*/\1/p" "$1" | head -1 | tr -d ' "'
}

# 驱动侧参数(与 point_lio 对比,C3/C4 用)
DRV_LIDAR_TOPIC=""; DRV_IMU_TOPIC=""; HOST_IP=""
if [ -n "$DRV_CFG" ]; then
  DRV_LIDAR_TOPIC="$(yaml_get "$DRV_CFG" lidar_topic)"
  DRV_IMU_TOPIC="$(yaml_get "$DRV_CFG" imu_topic)"
  HOST_IP="$(yaml_get "$DRV_CFG" host_ip)"
fi

LIDAR_TYPE="$(yaml_get "$PIO_CFG" lidar_type)"
TS_UNIT="$(yaml_get "$PIO_CFG" timestamp_unit)"
LID_TOPIC_CFG="$(yaml_get "$PIO_CFG" lid_topic)"
IMU_TOPIC_CFG="$(yaml_get "$PIO_CFG" imu_topic)"
BODY_PUB="$(yaml_get "$PIO_CFG" scan_bodyframe_pub_en)"
DET_RANGE="$(yaml_get "$PIO_CFG" det_range)"
SCAN_LINE="$(yaml_get "$PIO_CFG" scan_line)"

# ── C1 驱动包/config 与 point_lio 分支 ─────────────────────────────────────
head2 'C1  驱动包/config 与 point_lio 分支'
if [ -z "$DRV_CFG" ]; then
  fail "找不到 mid360_driver 的 config/param.yaml"
  info "确认 src/mid360_driver 已 vendor,或先 colcon build --packages-select mid360_driver"
elif [ -z "$DRV_LAUNCH" ]; then
  fail "找不到 mid360_driver 的 launch/mid360_driver.launch.py"
else
  pass "mid360_driver 包、config 与 launch 齐备"
fi
if [ "$LIDAR_TYPE" = "1" ]; then
  pass "point_lio preprocess.lidar_type=1(Livox handler)"
else
  fail "preprocess.lidar_type='$LIDAR_TYPE',应为 1(Livox)"
fi
if [ -z "$PREPROC" ]; then
  fail "找不到 src/point_lio_ros2/src/preprocess.cpp,无法确认 P1 补丁"
  info "设 MID360_WS 指向工作空间根目录后重试"
elif grep -q 'void Preprocess::livox_handler' "$PREPROC" \
     && grep -q 'PointCloud2ConstIterator' "$PREPROC"; then
  pass "livox_handler + PointCloud2ConstIterator 齐备(本地补丁 P1 在位)"
else
  fail "preprocess.cpp 缺少 livox_handler 或 PointCloud2ConstIterator(P1 不在位)"
  info "mid360_driver 的点云布局为 x/y/z/intensity(float32) + timestamp(float64,秒);"
  info "P1 必须用 PointCloud2ConstIterator 解析,不能再用 livox_ros::Point/fromROSMsg。"
  info "多半是同步上游时补丁被覆盖。见 docs/PATCHES.md 的 P1,恢复后再 build。"
fi

# ── C2 点时间单位 ──────────────────────────────────────────────────────────
head2 'C2  点时间单位 timestamp_unit'
if [ "$LIDAR_TYPE" = "1" ]; then
  if [ "$TS_UNIT" = "0" ]; then
    pass "timestamp_unit=0(秒):与 mid360_driver 的 float64 绝对 Unix 秒一致"
  else
    fail "timestamp_unit='$TS_UNIT',应为 0(秒)"
    info "mid360_driver 的 timestamp 字段是 float64 绝对 Unix 秒(offset 16)。"
    info "point_lio 按 timestamp_unit 把帧内时差缩放到 ms:0→×1e3,3→×1e-6;"
    info "若仍写 3(ns),curvature 会小 1000 倍 → 逐点 deskew 静默失效(不报错)。"
  fi
else
  pass "非 Livox 分支,跳过"
fi

# ── C3 话题名一致性(驱动 vs point_lio) ────────────────────────────────────
head2 'C3  topic 名一致性(驱动 vs point_lio)'
if [ -z "$DRV_CFG" ]; then
  warn "读不到驱动 param.yaml,跳过 topic 对比"
else
  if [ "$DRV_LIDAR_TOPIC" = "$LID_TOPIC_CFG" ]; then
    pass "lidar 话题一致:$DRV_LIDAR_TOPIC(mid360_driver = point_lio lid_topic)"
  else
    fail "lidar 话题不一致:mid360_driver='$DRV_LIDAR_TOPIC',point_lio lid_topic='$LID_TOPIC_CFG'"
    info "两者必须逐字相同(推荐 /livox/lidar);改 src/mid360_driver/config/param.yaml 或 point_lio yaml。"
  fi
  if [ "$DRV_IMU_TOPIC" = "$IMU_TOPIC_CFG" ]; then
    pass "imu 话题一致:$DRV_IMU_TOPIC(mid360_driver = point_lio imu_topic)"
  else
    fail "imu 话题不一致:mid360_driver='$DRV_IMU_TOPIC',point_lio imu_topic='$IMU_TOPIC_CFG'"
    info "两者必须逐字相同(推荐 /livox/imu)。"
  fi
fi

# ── C4 host_ip 绑定 + octomap 必须吃机体系点云 ─────────────────────────────
head2 'C4  host_ip 绑定 与 octomap 输入(机体系点云 + 单一真相源)'
if [ -z "$HOST_IP" ]; then
  warn "读不到驱动 param.yaml 的 host_ip(驱动 bind 会失败)"
elif ! command -v ip >/dev/null 2>&1; then
  warn "没有 ip 命令,无法核对 host_ip=$HOST_IP 是否属于本机网卡"
else
  LOCAL_IPS="$(ip -o -4 addr show 2>/dev/null | awk '{print $4}' | cut -d/ -f1)"
  if printf '%s\n' "$LOCAL_IPS" | grep -qx -- "$HOST_IP"; then
    pass "host_ip=$HOST_IP 是本机网卡地址"
  else
    warn "host_ip=$HOST_IP 不在本机网卡地址中($(printf '%s ' $LOCAL_IPS))"
    info "驱动只在 host_ip 上 bind UDP 56301/56401;填错会 bind 失败、收不到任何数据。"
    info "换机器/网卡时改 src/mid360_driver/config/param.yaml 的 host_ip,并重新 build 同步 install。"
  fi
fi
if [ "$BODY_PUB" = "true" ]; then
  pass "scan_bodyframe_pub_en=true,会发 /cloud_registered_body"
else
  fail "scan_bodyframe_pub_en='${BODY_PUB:-空}',应为 true"
  info "octomap_server 把 lookupTransform(frame_id, 点云 frame) 的平移当作射线起点;"
  info "喂世界系点云会让所有射线从世界原点射出,自由空间全错(地图看着有、其实是错的)。"
fi
if [ -n "$OCM_LAUNCH" ] && grep -q "cloud_registered_body" "$OCM_LAUNCH"; then
  pass "octomap launch 里有 cloud_in -> /cloud_registered_body 重映射"
else
  fail "octomap launch 里找不到 /cloud_registered_body 重映射"
fi
# 双真相源:resolution / max_range 不应再出现在 launch 的数字默认值里
dup="$(grep -nE "default_value=['\"][0-9]" "$OCM_LAUNCH" 2>/dev/null | grep -E "resolution|max_range" || true)"
if [ -n "$dup" ]; then
  warn "octomap launch 里又出现数字默认值(会盖掉 yaml):"
  printf '        %s\n' "$dup"
  info "resolution/max_range 的唯一真相应是 config/octomap_mid360.yaml;"
  info "launch 侧应保持 default_value=''(留空=用 yaml),见 docs/PATCHES.md 的 A 项说明。"
else
  pass "octomap 参数单一真相源(yaml),launch 不再覆盖"
fi
if [ -n "$OCM_CFG" ] && [ -n "$DET_RANGE" ]; then
  OCM_RANGE="$(yaml_get "$OCM_CFG" max_range)"
  case "$DET_RANGE" in
    100.0|100|"")
      info "提示:point_lio det_range=${DET_RANGE:-未设}(≈不限制),远处噪声点会进 SLAM;"
      info "      收到 20~30 可在预处理阶段就切掉,比只靠 octomap max_range(现 ${OCM_RANGE:-?})更省算力。"
      ;;
  esac
fi

# ── C5 src 与 install 是否同步 ─────────────────────────────────────────────
head2 'C5  src / install 一致性(改了 src 没 build 的常见坑)'
if [ -n "$WS" ]; then
  stale=""
  for pair in \
    "$WS/src/mid360_driver/config/param.yaml:$WS/install/mid360_driver/share/mid360_driver/config/param.yaml" \
    "$WS/src/mid360_driver/launch/mid360_driver.launch.py:$WS/install/mid360_driver/share/mid360_driver/launch/mid360_driver.launch.py" \
    "$WS/src/mid360_bringup/config/octomap_mid360.yaml:$WS/install/mid360_bringup/share/mid360_bringup/config/octomap_mid360.yaml" \
    "$WS/src/mid360_bringup/launch/octomap_mid360.launch.py:$WS/install/mid360_bringup/share/mid360_bringup/launch/octomap_mid360.launch.py" \
    "$WS/src/point_lio_ros2/config/mid360.yaml:$WS/install/point_lio/share/point_lio/config/mid360.yaml" ; do
    a="${pair%%:*}"; b="${pair##*:}"
    if [ -f "$a" ] && [ -f "$b" ] && ! cmp -s "$a" "$b"; then
      stale="$stale
        $a"
    fi
  done
  if [ -n "$stale" ]; then
    warn "src 与 install 里的副本不一致(运行时读的是 install):$stale"
    info "跑 colcon build --packages-select mid360_driver mid360_bringup point_lio 同步"
  else
    pass "关心的配置在 src 与 install 中一致"
  fi

  so="$(ls "$WS"/install/point_lio/lib/*.so 2>/dev/null | head -1)"
  if [ -n "$so" ] && [ -d "$WS/src/point_lio_ros2/src" ]; then
    newer="$(find "$WS/src/point_lio_ros2/src" "$WS/src/point_lio_ros2/include" \
                   \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \) -newer "$so" 2>/dev/null | head -3)"
    if [ -n "$newer" ]; then
      warn "有源码比已安装的 point_lio 库更新(需 colcon build):"
      printf '        %s\n' $newer
    else
      pass "point_lio 库不比源码旧"
    fi
  fi
fi

# ── C6 point_lio launch 硬编码是否盖住了 yaml ────────────────────────────
head2 'C6  point_lio launch / yaml 双真相源'
if [ -n "$PIO_LAUNCH" ]; then
  # launch 参数字典里的键:'name':(先排除注释行,免得把说明文字里的键当成真的)
  launch_keys="$(grep -v '^[[:space:]]*#' "$PIO_LAUNCH" \
                 | grep -oE "'[a-z_][a-z0-9_]*'[[:space:]]*:" | tr -d "': " | sort -u)"
  # yaml 的顶层键(该文件里 8 空格缩进)
  yaml_keys="$(grep -E '^ {8}[a-z_][a-z0-9_]*:' "$PIO_CFG" | cut -d: -f1 | tr -d ' ' | sort -u)"
  if [ -z "$yaml_keys" ]; then
    warn "没能从 $PIO_CFG 抽出顶层键,双真相源检查未生效(脚本需修)"
  else
    both="$(comm -12 <(printf '%s\n' "$launch_keys") <(printf '%s\n' "$yaml_keys") | tr '\n' ' ')"
    # 约定 A 允许的「条件覆盖」:launch 里声明了 default_value='' 的同名 launch 参数
    # (P7 的 pcd_save 就是这种:只有显式传参才追加参数项,不传时求值为空串、不追加,
    #  yaml 仍是真相源)。这类不该报 WARN —— 否则真告警会被常年淹没。
    # 注意这是启发式:认的是「声明了空默认值」这个标记,所以覆盖必须真的写成
    # OpaqueFunction + if 判断;不要又把名字无条件塞回参数字典(那就回到 P5 的坑)。
    cond_keys="$(grep -E "^[[:space:]]*'[a-z_][a-z0-9_]*',[[:space:]]*default_value=''" "$PIO_LAUNCH" \
                 | grep -oE "'[a-z_][a-z0-9_]*'" | tr -d "'" | sort -u)"
    both_list="$(printf '%s\n' "$both" | tr ' ' '\n' | grep . | sort -u)"
    hard_both="$(comm -23 <(printf '%s\n' "$both_list") <(printf '%s\n' "$cond_keys") | tr '\n' ' ' | sed 's/ *$//')"
    cond_both="$(comm -12 <(printf '%s\n' "$both_list") <(printf '%s\n' "$cond_keys") | tr '\n' ' ' | sed 's/ *$//')"
    n=$(printf '%s\n' "$launch_keys" | grep -c . || true)
    if [ -n "$hard_both" ]; then
      warn "这些参数同时写在 launch 和 yaml 里,launch 会赢(改 yaml 无效): $hard_both"
      info "改值请改 launch(file: $PIO_LAUNCH);或按 docs/PATCHES.md 约定 A 从 launch 里删掉。"
    elif [ -n "$cond_both" ]; then
      pass "launch 硬编码 $n 项均不同名于 yaml;另有条件覆盖(约定 A,不传参时 yaml 生效): $cond_both"
    else
      pass "launch 未覆盖 yaml 的同名参数(launch 硬编码 $n 项,均不在 yaml 里)"
    fi
  fi
else
  pass "未找到 point_lio 的 mapping_mid360.launch.py,跳过"
fi

# ── R1 运行时:实际 topic 类型 ─────────────────────────────────────────────
if [ "$RUNTIME" = 1 ]; then
  head2 'R1  运行时 topic 类型'
  if ! command -v ros2 >/dev/null 2>&1; then
    fail "找不到 ros2 命令(先 source /opt/ros/<distro>/setup.bash)"
  else
    EXPECT_TYPE="sensor_msgs/msg/PointCloud2"
    printf '        等待 %s(最多 %ss)…\n' "$LID_TOPIC" "$TIMEOUT"
    t=0
    while [ "$t" -lt "$TIMEOUT" ]; do
      if timeout 5 ros2 topic list 2>/dev/null | grep -qx -- "$LID_TOPIC"; then break; fi
      sleep 1; t=$((t + 1))
    done
    if ! timeout 5 ros2 topic list 2>/dev/null | grep -qx -- "$LID_TOPIC"; then
      fail "$LID_TOPIC 在 $TIMEOUT s 内没出现(驱动没起来?topic 名不对?)"
    else
      ACT_TYPE="$(timeout 10 ros2 topic info -v "$LID_TOPIC" 2>/dev/null \
                  | grep -m1 -E '^(Topic )?Type:' | awk '{print $NF}')"
      if [ -z "$ACT_TYPE" ]; then
        warn "拿不到 $LID_TOPIC 的类型(还没 publisher?)"
      elif [ "$ACT_TYPE" != "$EXPECT_TYPE" ]; then
        fail "$LID_TOPIC 实际是 $ACT_TYPE,mid360_driver 固定发 $EXPECT_TYPE"
        info "检查驱动是否真的启动、参数是否正确;订阅类型错配时 point_lio 会静默收不到点云。"
      else
        pass "$LID_TOPIC 类型 = $ACT_TYPE"
      fi
    fi
    IACT="$(timeout 10 ros2 topic info -v /livox/imu 2>/dev/null \
            | grep -m1 -E '^(Topic )?Type:' | awk '{print $NF}')"
    if [ "$IACT" = "sensor_msgs/msg/Imu" ]; then
      pass "/livox/imu 类型 = $IACT"
    else
      fail "/livox/imu 类型 = $IACT(为空=没读到),应为 sensor_msgs/msg/Imu"
    fi
    if command -v ss >/dev/null 2>&1; then
      PORTS="$(ss -lunp 2>/dev/null | grep -E ':(56301|56401)\b' || true)"
      if [ -n "$PORTS" ]; then
        pass "UDP 56301/56401 已绑定(驱动正在收流)"
        printf '%s\n' "$PORTS" | sed 's/^/        /'
      else
        info "UDP 56301/56401 当前未被占用(若在 launch 前执行属正常)"
      fi
    fi
  fi
fi

# ── 汇总 ───────────────────────────────────────────────────────────────────
printf '\n'
if [ "$FAIL" -gt 0 ]; then
  printf '%spreflight 未通过:FAIL=%d WARN=%d%s\n' "$C_R" "$FAIL" "$WARN" "$C_0"
  exit 1
fi
printf '%spreflight 通过%s(WARN=%d)\n' "$C_G" "$C_0" "$WARN"
exit 0
