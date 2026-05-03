cmake_minimum_required(VERSION 3.8)
project(humanoid_manipulation_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ControlTrajectory.action"
  "action/ControlJointTrajectory.action"
  "action/VisualServoingArm.action"
  "srv/PlanTrajectory.srv"
  "srv/PlanJointTrajectory.srv"
  DEPENDENCIES geometry_msgs nav_msgs trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
