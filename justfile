# nav-ws 常用工作流
set shell := ["bash", "-cu"]
ros_distro := env_var_or_default("ROS_DISTRO", "jazzy")
ws := justfile_directory()

default:
    @just --list

build *args:
    bash -c 'source /opt/ros/{{ros_distro}}/setup.bash && cd {{ws}} && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release {{args}}'

clean:
    rm -rf {{ws}}/build {{ws}}/install {{ws}}/log

run *args:
    bash -c 'source /opt/ros/{{ros_distro}}/setup.bash && source {{ws}}/install/setup.bash && ros2 launch mid360_bringup mid360.launch.py {{args}}'

octomap *args:
    bash -c 'source /opt/ros/{{ros_distro}}/setup.bash && source {{ws}}/install/setup.bash && ros2 launch mid360_bringup octomap_mid360.launch.py {{args}}'

backend *args:
    bash -c 'source /opt/ros/{{ros_distro}}/setup.bash && source {{ws}}/install/setup.bash && ros2 launch point_lio_backend backend_mid360.launch.py {{args}}'

pcd-view *args:
    bash -c 'source /opt/ros/{{ros_distro}}/setup.bash && source {{ws}}/install/setup.bash && ros2 launch mid360_bringup pcd_view.launch.py {{args}}'

preflight *args:
    bash {{ws}}/src/mid360_bringup/scripts/preflight.sh {{args}}

check-tf *args:
    bash {{ws}}/src/mid360_bringup/scripts/check_tf.sh {{args}}

cleanup:
    bash {{ws}}/src/mid360_bringup/scripts/cleanup.sh

test-backend:
    bash -c 'source /opt/ros/{{ros_distro}}/setup.bash && cd {{ws}} && colcon test --packages-select point_lio_backend && colcon test-result --verbose'

record *args:
    bash -c 'source /opt/ros/{{ros_distro}}/setup.bash && mkdir -p {{ws}}/bags && cd {{ws}}/bags && \
    topics="/livox/lidar /livox/imu"; \
    timeout 2 ros2 topic hz /camera/camera/color/image_raw --window 3 >/dev/null 2>&1 && topics="$topics /camera/camera/color/image_raw /camera/camera/color/camera_info"; \
    echo "录制话题: $topics"; \
    ros2 bag record -s mcap --compression-mode file --compression-format zstd $topics {{args}} -o bag_$(date +%Y%m%d_%H%M%S)'

realsense-setup:
    bash -c 'raw="https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules"; \
    for m in "" https://ghfast.top/ https://gh-proxy.com/ https://ghproxy.net/; do \
      wget -q --timeout=10 -O /tmp/99-realsense-libusb.rules "$m$raw" && grep -q "RealSense" /tmp/99-realsense-libusb.rules && { echo "via ${m:-direct}"; break; }; \
    done; grep -q "RealSense" /tmp/99-realsense-libusb.rules || { echo "所有源都失败" >&2; exit 1; }; \
    sudo cp /tmp/99-realsense-libusb.rules /etc/udev/rules.d/ && \
    sudo udevadm control --reload-rules && sudo udevadm trigger && \
    sudo usermod -aG plugdev,video $USER && echo "OK: 拔插 D435i 后重新登录(组生效)"'

