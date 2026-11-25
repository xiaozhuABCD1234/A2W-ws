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
    bash -c 'source /opt/ros/{{ros_distro}}/setup.bash && mkdir -p {{ws}}/bags && cd {{ws}}/bags && ros2 bag record /livox/lidar /livox/imu {{args}} -o bag_$(date +%Y%m%d_%H%M%S)'
