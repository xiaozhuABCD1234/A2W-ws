cmake_minimum_required(VERSION 3.16)
project(planner CXX)

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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_model REQUIRED)

add_library(planner_core src/search.cpp src/footprint_checker.cpp)
target_include_directories(planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${robot_model_INCLUDE_DIRS})
target_link_libraries(planner_core ${robot_model_LIBRARIES})

add_executable(planner_node src/planner_node.cpp)
target_link_libraries(planner_node planner_core ${robot_model_LIBRARIES})
ament_target_dependencies(planner_node rclcpp nav_msgs geometry_msgs sensor_msgs tf2 tf2_ros)

install(TARGETS planner_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS planner_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_search test/test_search.cpp)
  target_link_libraries(test_search planner_core)
  ament_add_gtest(test_footprint_checker test/test_footprint_checker.cpp)
  target_link_libraries(test_footprint_checker planner_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(planner_core)
ament_package()
