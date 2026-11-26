cmake_minimum_required(VERSION 3.16)
project(ground_segmentation)

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
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP QUIET)

add_library(ground_segmentation_core SHARED src/segmenter.cpp)
target_include_directories(ground_segmentation_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_directories(ground_segmentation_core PUBLIC ${PCL_LIBRARY_DIRS})
target_link_libraries(ground_segmentation_core PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})
target_compile_definitions(ground_segmentation_core PUBLIC ${PCL_DEFINITIONS})
if(OpenMP_CXX_FOUND)
  target_link_libraries(ground_segmentation_core PUBLIC OpenMP::OpenMP_CXX)
endif()

add_executable(ground_segmentation_node src/ground_segmentation_node.cpp)
target_link_libraries(ground_segmentation_node ground_segmentation_core)
ament_target_dependencies(ground_segmentation_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
  pcl_conversions
)

install(TARGETS ground_segmentation_core ground_segmentation_node
  EXPORT ground_segmentation_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(ground_segmentation_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 PCL)
ament_package()
