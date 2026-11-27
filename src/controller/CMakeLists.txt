cmake_minimum_required(VERSION 3.16)
project(controller CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(esdf REQUIRED)
find_package(robot_model REQUIRED)

add_library(controller_core
  src/path.cpp
  src/grid.cpp
  src/smoother.cpp
  src/pure_pursuit.cpp
  src/mppi.cpp)
target_include_directories(controller_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${esdf_INCLUDE_DIRS}
  ${robot_model_INCLUDE_DIRS})
target_link_libraries(controller_core ${esdf_LIBRARIES} ${robot_model_LIBRARIES})

add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node controller_core)
ament_target_dependencies(controller_node
  rclcpp nav_msgs geometry_msgs sensor_msgs visualization_msgs tf2 tf2_ros esdf robot_model)

install(TARGETS controller_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS controller_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_path test/test_path.cpp)
  target_link_libraries(test_path controller_core)
  ament_add_gtest(test_grid test/test_grid.cpp)
  target_link_libraries(test_grid controller_core)
  ament_add_gtest(test_smoother test/test_smoother.cpp)
  target_link_libraries(test_smoother controller_core)
  ament_add_gtest(test_pure_pursuit test/test_pure_pursuit.cpp)
  target_link_libraries(test_pure_pursuit controller_core)
  ament_add_gtest(test_mppi test/test_mppi.cpp)
  target_link_libraries(test_mppi controller_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(controller_core)
ament_package()
