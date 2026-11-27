cmake_minimum_required(VERSION 3.16)
project(filter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(filter_core SHARED
  src/voxel_filter.cpp
  src/pedestrian_filter.cpp
  src/self_filter.cpp
  src/detection_gate.cpp
)
target_include_directories(filter_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(filter_core PUBLIC Eigen3::Eigen)

add_executable(filter_node src/filter_node.cpp)
target_link_libraries(filter_node filter_core)
ament_target_dependencies(filter_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_srvs
  visualization_msgs
)

install(TARGETS filter_core filter_node
  EXPORT filter_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_filter test/test_filter.cpp)
  target_link_libraries(test_filter filter_core)
  ament_add_gtest(test_self_filter test/test_self_filter.cpp)
  target_link_libraries(test_self_filter filter_core)
  ament_add_gtest(test_detection_gate test/test_detection_gate.cpp)
  target_link_libraries(test_detection_gate filter_core)
endif()

ament_export_targets(filter_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Eigen3)
ament_package()
