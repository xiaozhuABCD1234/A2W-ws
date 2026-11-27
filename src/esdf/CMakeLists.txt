cmake_minimum_required(VERSION 3.16)
project(esdf CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)

add_library(esdf_core src/edt.cpp)
target_include_directories(esdf_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(esdf_node src/esdf_node.cpp)
target_include_directories(esdf_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(esdf_node esdf_core ${PCL_LIBRARIES})
ament_target_dependencies(esdf_node
  rclcpp nav_msgs sensor_msgs geometry_msgs std_msgs pcl_conversions)

install(TARGETS esdf_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_edt test/test_edt.cpp)
  target_link_libraries(test_edt esdf_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(esdf_core)
ament_package()
