cmake_minimum_required(VERSION 3.16)
project(relocalization)

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
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP QUIET)

add_library(relocalization_core SHARED src/relocator.cpp)
target_include_directories(relocalization_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_directories(relocalization_core PUBLIC ${PCL_LIBRARY_DIRS})
target_link_libraries(relocalization_core PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})
target_compile_definitions(relocalization_core PUBLIC ${PCL_DEFINITIONS})
if(OpenMP_CXX_FOUND)
  target_link_libraries(relocalization_core PUBLIC OpenMP::OpenMP_CXX)
endif()

add_executable(relocalization_node src/relocalization_node.cpp)
target_link_libraries(relocalization_node relocalization_core)
ament_target_dependencies(relocalization_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_srvs
  pcl_conversions
  tf2
  tf2_eigen
)

install(TARGETS relocalization_core relocalization_node
  EXPORT relocalization_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(relocalization_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 PCL)
ament_package()
