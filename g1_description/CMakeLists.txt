cmake_minimum_required(VERSION 3.30)
project(g1_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY assets launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
