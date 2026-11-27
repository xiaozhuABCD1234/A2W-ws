cmake_minimum_required(VERSION 3.16)
project(robot_model C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_program(XACRO_EXECUTABLE xacro)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/RobotProfile.msg
  DEPENDENCIES geometry_msgs std_msgs)

add_library(robot_model_core src/footprint.cpp src/limits.cpp)
target_include_directories(robot_model_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(robot_model_node src/robot_model_node.cpp)
target_link_libraries(robot_model_node robot_model_core)
ament_target_dependencies(robot_model_node rclcpp geometry_msgs std_msgs tf2_ros)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(robot_model_node "${cpp_typesupport_target}")

install(TARGETS robot_model_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS robot_model_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

install(FILES urdf/warehouse_agv/warehouse_agv.urdf
  DESTINATION share/${PROJECT_NAME}/urdf/warehouse_agv)
install(FILES urdf/warehouse_agv/LICENSE
  DESTINATION share/${PROJECT_NAME}/urdf)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_footprint test/test_footprint.cpp)
  target_link_libraries(test_footprint robot_model_core)
  ament_add_gtest(test_limits test/test_limits.cpp)
  target_link_libraries(test_limits robot_model_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(robot_model_core)
ament_package()
