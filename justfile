set shell := ["bash", "-cu"]
ros_distro := env_var_or_default("ROS_DISTRO", "jazzy")
ws := justfile_directory()

default:
    @just --list

build *args:
    bash -c 'source /opt/ros/{{ros_distro}}/setup.bash && cd {{ws}} && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release {{args}}'

clean:
    rm -rf {{ws}}/build {{ws}}/install {{ws}}/log

record *args:
    bash -c 'source /opt/ros/{{ros_distro}}/setup.bash && source {{ws}}/install/setup.bash && \
    mkdir -p {{ws}}/bags && cd {{ws}}/bags && \
    ros2 launch bringup record.launch.py {{args}}'

slam *args:
    bash -c 'source /opt/ros/{{ros_distro}}/setup.bash && source {{ws}}/install/setup.bash && \
    cd {{ws}} && ros2 launch bringup slam.launch.py {{args}}'

alias run := slam

replay bag="" rate="1.0" *args:
    bash -c 'source /opt/ros/{{ros_distro}}/setup.bash && source {{ws}}/install/setup.bash && \
    cd {{ws}} && b="{{bag}}"; b="${b#bag=}"; \
    [ -n "$b" ] || b=$(ls -dt bags/*/ 2>/dev/null | head -1); \
    [ -n "$b" ] || { echo "bags/ 下没有 bag" >&2; exit 1; }; \
    r="{{rate}}"; r="${r#rate=}"; \
    case "$r" in ""|*[!0-9.]*) echo "rate 必须是数字，例如 rate=2 或 1.5（收到 {{rate}}）" >&2; exit 1;; esac; \
    echo "replay $b rate=$r"; \
    ( trap - INT; exec ros2 launch bringup slam.launch.py use_driver:=false use_sim_time:=true {{args}} ) & \
    LP=$!; trap "kill -INT $LP 2>/dev/null; wait $LP 2>/dev/null" EXIT; \
    sleep 8 && ros2 bag play "$b" --clock --rate "$r"'

save-map out="PCD/backend_occ" topic="/occupancy_map_2d/map":
    bash -c 'source /opt/ros/{{ros_distro}}/setup.bash && p="{{out}}"; p="${p#out=}"; case "$p" in /*) ;; *) p="{{ws}}/$p";; esac; \
    t="{{topic}}"; t="${t#topic=}"; \
    mkdir -p "$(dirname "$p")" && ros2 run nav2_map_server map_saver_cli -t "$t" -f "$p" --ros-args -p save_map_timeout:=10.0'
