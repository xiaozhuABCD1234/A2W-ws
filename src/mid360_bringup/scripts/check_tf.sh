#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# check_tf.sh —— 一键启动后"rviz 里看不到东西 / Could not transform"的定位工具
#
# 用法(先 source 工作空间):
#   bash src/mid360_bringup/scripts/check_tf.sh
#   bash src/mid360_bringup/scripts/check_tf.sh --frames 8
#
# 做四件事:
#   1) 列出链路上的节点,确认后端/前端都在;
#   2) 采 /tf n 秒,汇总所有 "父帧 -> 子帧" 边(有 map->camera_init 才能用 map 系);
#   3) 逐个打印 rviz/octomap 关心的话题的 frame_id 与频率;
#   4) 用「当前时间」查一次 map->camera_init(和 rviz 的判定方式一致),给出结论。
#
# 退出码:0 正常 / 1 有问题(会打印可操作的下一步)
# ─────────────────────────────────────────────────────────────────────────────
set -uo pipefail

FRAMES=5
while [ $# -gt 0 ]; do
  case "$1" in
    --frames) FRAMES="${2:-5}"; shift ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "未知参数:$1" >&2; exit 2 ;;
  esac
  shift
done

if [ -t 1 ]; then
  R=$'\033[31m'; G=$'\033[32m'; Y=$'\033[33m'; D=$'\033[2m'; Z=$'\033[0m'
else R=''; G=''; Y=''; D=''; Z=''; fi
ok()   { printf '  %sOK%s    %s\n' "$G" "$Z" "$1"; }
bad()  { printf '  %sFAIL%s  %s\n' "$R" "$Z" "$1"; }
warn() { printf '  %sWARN%s  %s\n' "$Y" "$Z" "$1"; }
note() { printf '        %s%s%s\n' "$D" "$1" "$Z"; }

command -v ros2 >/dev/null || { echo "找不到 ros2,先 source /opt/ros/<distro>/setup.bash"; exit 2; }

FAIL=0

echo
echo "── 1. 节点 ──────────────────────────────────────────────────────────────"
NODES="$(timeout 8 ros2 node list 2>/dev/null | sort)"
printf '%s\n' "$NODES" | sed 's/^/        /'
for n in /mid360_driver /laserMapping /point_lio_backend /octomap_server; do
  if printf '%s\n' "$NODES" | grep -qx -- "$n"; then
    ok "节点 $n 在跑"
  else
    case "$n" in
      /octomap_server) warn "节点 $n 不在(没装 octomap_server 或 use_octomap:=false 属正常)" ;;
      *) bad "节点 $n 不在跑"; FAIL=1 ;;
    esac
  fi
done

echo
echo "── 2. TF 树(采 ${FRAMES}s)─────────────────────────────────────────────────"
EDGES="$(TF_SECONDS="$FRAMES" timeout $((FRAMES + 6)) /usr/bin/python3 - <<'PY' 2>/dev/null || true
import os, time
try:
    import rclpy
    from tf2_msgs.msg import TFMessage
except Exception:
    raise SystemExit(0)
rclpy.init()
node = rclpy.create_node('tf_edge_probe')
seen = set()

def cb(msg):
    for t in msg.transforms:
        seen.add((t.header.frame_id, t.child_frame_id))

node.create_subscription(TFMessage, '/tf', cb, 50)
end = time.time() + float(os.environ.get('TF_SECONDS', '5'))
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
for parent, child in sorted(seen):
    print('{} -> {}'.format(parent, child))
node.destroy_node()
rclpy.shutdown()
PY
)"
if [ -z "$EDGES" ]; then
  bad "没收到任何 /tf(前端 point_lio 没起来?或 TF 发布失败)"
  FAIL=1
else
  while read -r line; do [ -n "$line" ] && ok "$line"; done <<< "$EDGES"
  if printf '%s\n' "$EDGES" | grep -qx 'map -> camera_init'; then
    ok "map -> camera_init 存在(backend 在发纠偏 TF)"
  else
    bad "缺少 map -> camera_init:所有 map 系显示都会报 Could not transform"
    note "后端没起? ros2 node list 里要有 /point_lio_backend"
    note "后端起了但一直打印 WARN「尚未收到 /aft_mapped_to_init」→ 前端没出位姿"
    FAIL=1
  fi
fi

echo
echo "── 3. 话题 frame_id / 频率 ──────────────────────────────────────────────"
check_topic() {
  local t="$1" want="$2" hz fid
  fid="$(timeout 6 ros2 topic echo --once "$t" --field header.frame_id 2>/dev/null | head -1 | tr -d ' ')"
  if [ -z "$fid" ]; then
    warn "$t 没有消息(或 header.frame_id 为空)"
    return
  fi
  hz="$(timeout 5 ros2 topic hz "$t" 2>&1 | grep -m1 'average rate' | awk '{print $3}')"
  if [ "$fid" = "$want" ]; then
    ok "$t  frame_id=$fid  频率=$hz"
  else
    bad "$t  frame_id=$fid(期望 $want)"
    note "Fixed Frame 是 map 时,frame 不是 map 的显示都要 map->camera_init 变换"
    FAIL=1
  fi
}
check_topic /backend/map_points map
check_topic /backend/path map
check_topic /octomap_point_cloud_centers map
check_topic /cloud_registered camera_init
check_topic /cloud_registered_body body

echo
echo "── 4. 按「当前时间」查 map -> camera_init(与 rviz 判定一致)───────────────"
# 注意:必须先把 tf2_echo 输出落到文件再 grep。管道 + grep -q 命中后立刻关闭读端,
# 而 tf2_echo 走管道时是全缓冲 —— 直接 grep -q 会假失败(实测踩过)。
TF_LOG="$(mktemp)"
timeout 10 stdbuf -oL -eL ros2 run tf2_ros tf2_echo map camera_init >"$TF_LOG" 2>&1 || true
if grep -q 'At time' "$TF_LOG"; then
  ok "tf2_echo map -> camera_init 成功"
else
  bad "tf2_echo map -> camera_init 失败"
  note "若 §2 有这条边、这里却失败 → 时间戳落在 rviz 缓冲窗口外"
  sed -n '1,6p' "$TF_LOG" | sed 's/^/        /'
  FAIL=1
fi
rm -f "$TF_LOG"

echo
if [ "$FAIL" -gt 0 ]; then
  printf '%s结论:有问题(见上面的 FAIL)%s\n' "$R" "$Z"
  note "把整段输出贴回来即可,不用自己先判断。"
  exit 1
fi
printf '%s结论:TF 链路正常%s\n' "$G" "$Z"
note "若 rviz 仍报 Could not transform:在左侧 Displays 面板看是哪个显示项"
note "(多半是打开了 frame 为 camera_init 的显示:/cloud_registered、/path)"
note "也可能是你在终端里另起了一个 rviz2 —— 它恢复的是上次会话的 Fixed Frame。"
exit 0
