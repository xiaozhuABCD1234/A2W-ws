cmake_minimum_required(VERSION 3.8)
project(mid360_bringup)

find_package(ament_cmake REQUIRED)

# 纯 launch / config / rviz 包,没有编译目标
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()