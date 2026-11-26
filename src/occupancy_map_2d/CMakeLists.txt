cmake_minimum_required(VERSION 3.16)
project(occupancy_map_2d)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_library(occupancy_map_2d_core SHARED src/grid.cpp)
target_include_directories(occupancy_map_2d_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_directories(occupancy_map_2d_core PUBLIC ${PCL_LIBRARY_DIRS})
target_link_libraries(occupancy_map_2d_core PUBLIC Eigen3::Eigen ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
target_compile_definitions(occupancy_map_2d_core PUBLIC ${PCL_DEFINITIONS})
ament_target_dependencies(occupancy_map_2d_core PUBLIC rclcpp nav_msgs)

add_executable(occupancy_map_2d_node src/occupancy_map_2d_node.cpp)
target_link_libraries(occupancy_map_2d_node occupancy_map_2d_core)
ament_target_dependencies(occupancy_map_2d_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
  std_srvs
  pcl_conversions
  tf2
  tf2_ros
  tf2_eigen
)

install(TARGETS occupancy_map_2d_core occupancy_map_2d_node
  EXPORT occupancy_map_2d_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(occupancy_map_2d_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp nav_msgs Eigen3 PCL OpenCV)
ament_package()
