#!/usr/bin/env bash
# 清理 mid360 链路残留进程。driver 被 Ctrl-C 后可能残留,占住雷达的
# UDP 56301/56401,新驱动 bind failed → point_lio 无点云/无 tf → rviz 报错。
# 报错时先跑这个再重启。只杀本链路进程;[x] 写法避免 pkill 匹配到本脚本自身。

pkill -9 -f 'mid360_driver_nod[e]'     2>/dev/null   # mid360 驱动
pkill -9 -f 'pointlio_mappi[n]g'       2>/dev/null   # point_lio 建图
pkill -9 -x rviz2                      2>/dev/null   # rviz2

sleep 1
left=$(ps -eo cmd | grep -cE 'mid360_driver_nod[e]|pointlio_mappi[n]g' || true)
if [ "$left" -eq 0 ]; then
    echo "清理完成,重新启动: ros2 launch bringup slam.launch.py"
else
    echo "还有 $left 个残留进程,请人工核对: ps -eo pid,cmd | grep -E 'mid360_driver|pointlio'"
fi