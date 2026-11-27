cmake_minimum_required(VERSION 3.16)
project(costmap_2d CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(costmap_core src/costmap.cpp)
target_include_directories(costmap_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR})
target_link_libraries(costmap_core PUBLIC ${PCL_LIBRARIES})
ament_target_dependencies(costmap_core PUBLIC rclcpp nav_msgs)

add_executable(costmap_2d_node src/costmap_node.cpp)
target_link_libraries(costmap_2d_node costmap_core)
ament_target_dependencies(costmap_2d_node
  rclcpp nav_msgs sensor_msgs std_srvs tf2 tf2_ros tf2_eigen pcl_conversions)

install(TARGETS costmap_2d_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_costmap test/test_costmap.cpp)
  target_link_libraries(test_costmap costmap_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(costmap_core)
ament_package()
