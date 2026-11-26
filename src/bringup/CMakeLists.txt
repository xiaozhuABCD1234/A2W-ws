cmake_minimum_required(VERSION 3.8)
project(bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/cleanup.sh
  scripts/preflight.sh
  scripts/check_tf.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
