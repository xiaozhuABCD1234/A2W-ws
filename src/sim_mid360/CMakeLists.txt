cmake_minimum_required(VERSION 3.16)
project(sim_mid360 CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz-plugin REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz-msgs REQUIRED)

add_executable(gz_bridge_node src/gz_bridge_node.cpp)
target_include_directories(gz_bridge_node SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(gz_bridge_node gz-transport::core gz-msgs::core)
ament_target_dependencies(gz_bridge_node rclcpp sensor_msgs geometry_msgs rosgraph_msgs)

add_library(gz-sim-mid360-pattern-system SHARED src/mid360_pattern_system.cpp)
target_link_libraries(gz-sim-mid360-pattern-system gz-sim::core gz-plugin::loader
  gz-transport::core gz-msgs::core)
set_target_properties(gz-sim-mid360-pattern-system PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

install(TARGETS gz-sim-mid360-pattern-system
  LIBRARY DESTINATION lib)

install(TARGETS gz_bridge_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch world models
  DESTINATION share/${PROJECT_NAME})

ament_package()
