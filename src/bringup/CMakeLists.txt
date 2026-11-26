cmake_minimum_required(VERSION 3.8)
project(bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/cleanup.sh
  scripts/preflight.sh
  scripts/check_tf.sh
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(map_watchdog src/map_watchdog.cpp)
target_link_libraries(map_watchdog ${OCTOMAP_LIBRARIES})
target_include_directories(map_watchdog PRIVATE ${OCTOMAP_INCLUDE_DIRS})
ament_target_dependencies(map_watchdog rclcpp nav_msgs octomap_msgs)
install(TARGETS map_watchdog
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
