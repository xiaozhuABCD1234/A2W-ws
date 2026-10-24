cmake_minimum_required(VERSION 3.16)
project(point_lio_backend)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 无 ROS 依赖的算法核心，方便 gtest 直接链接。
add_library(point_lio_backend_core STATIC
  src/keyframe_manager.cpp
  src/scan_context.cpp
  src/loop_detector.cpp
  src/pose_graph.cpp
)
target_include_directories(point_lio_backend_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(point_lio_backend_core PUBLIC
  Eigen3::Eigen
  Ceres::ceres
  ${PCL_LIBRARIES}
)

add_executable(point_lio_backend_node src/backend_node.cpp)
target_link_libraries(point_lio_backend_node point_lio_backend_core)
ament_target_dependencies(point_lio_backend_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_ros
  pcl_conversions
  visualization_msgs
  diagnostic_msgs
  std_srvs
)

install(TARGETS point_lio_backend_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scan_context test/test_scan_context.cpp)
  target_link_libraries(test_scan_context point_lio_backend_core)
  ament_add_gtest(test_pose_graph test/test_pose_graph.cpp)
  target_link_libraries(test_pose_graph point_lio_backend_core)
  ament_add_gtest(test_keyframe_manager test/test_keyframe_manager.cpp)
  target_link_libraries(test_keyframe_manager point_lio_backend_core)
endif()

ament_package()
