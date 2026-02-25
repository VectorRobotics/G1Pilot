cmake_minimum_required(VERSION 3.8)
project(g1_pilot)

option(BUILD_WITH_CASADI_SUPPORT "Build the library with the support of CASADI" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pinocchio REQUIRED)

# ---- CASADi setup ---- #
# Add CASADi
add_subdirectory(third_party/casadi)

# Treat CASADi headers as system includes to suppress warnings
get_target_property(_casadi_includes casadi INTERFACE_INCLUDE_DIRECTORIES)
if(_casadi_includes)
    set_target_properties(casadi PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_casadi_includes}")
endif()

# ---- G1 Pilot ---- #
add_compile_options(-Wall -Wextra -Wpedantic)

file(GLOB_RECURSE g1_pilot_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/base/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/arm_ik/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/arm_control/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/arm_mp/*.cpp"
)

add_library(g1_pilot
    ${g1_pilot_SOURCES}
)

target_compile_definitions(g1_pilot PUBLIC
    COAL_DISABLE_HPP_FCL_WARNINGS
    $<$<BOOL:${BUILD_WITH_CASADI_SUPPORT}>:USE_CASADI>
)

target_include_directories(g1_pilot
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    SYSTEM PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/casadi>
)

target_link_libraries(g1_pilot
    PUBLIC
        Eigen3::Eigen
        pinocchio::pinocchio
        casadi
)

# ---- ROS 2 Nodes ---- #

add_executable(ik_joint_state_publisher src/joint_state_publisher.cpp)
add_executable(impede_arms src/impede_arms.cpp)
add_executable(joint_traj_publisher src/joint_traj_publisher.cpp)
add_executable(visual_servo src/visual_servo.cpp)
add_executable(grav_ff src/grav_ff.cpp)
add_executable(visual_servo_ik src/visual_servo_ik.cpp)

target_link_libraries(ik_joint_state_publisher g1_pilot)
ament_target_dependencies(ik_joint_state_publisher ament_index_cpp rclcpp sensor_msgs nav_msgs)

target_link_libraries(impede_arms g1_pilot)
ament_target_dependencies(impede_arms ament_index_cpp rclcpp sensor_msgs nav_msgs)

target_link_libraries(joint_traj_publisher g1_pilot)
ament_target_dependencies(joint_traj_publisher ament_index_cpp rclcpp nav_msgs sensor_msgs)

target_link_libraries(visual_servo g1_pilot)
ament_target_dependencies(visual_servo ament_index_cpp rclcpp nav_msgs sensor_msgs)

target_link_libraries(grav_ff g1_pilot)
ament_target_dependencies(grav_ff ament_index_cpp rclcpp sensor_msgs)

target_link_libraries(visual_servo_ik g1_pilot)
ament_target_dependencies(visual_servo_ik ament_index_cpp rclcpp sensor_msgs nav_msgs geometry_msgs)

# ---- Testing ---- #

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(GTest REQUIRED)

    add_executable(test_ik test/arm_ik/test_ik.cpp)
    target_link_libraries(test_ik PRIVATE g1_pilot GTest::gtest_main)
    target_compile_definitions(test_ik PRIVATE USE_CASADI)

    add_executable(test_control test/arm_control/test_control.cpp)
    target_link_libraries(test_control PRIVATE g1_pilot GTest::gtest_main)

    add_executable(test_mp_ik test/arm_mp/test_traj_gen.cpp)
    target_link_libraries(test_mp_ik PRIVATE g1_pilot GTest::gtest_main)
    target_compile_definitions(test_mp_ik PRIVATE USE_CASADI)
endif()

# ---- Installation ---- #

install(TARGETS g1_pilot
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
)

install(TARGETS
    ik_joint_state_publisher
    impede_arms
    joint_traj_publisher
    visual_servo
    grav_ff
    visual_servo_ik
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch assets rviz DESTINATION share/${PROJECT_NAME})

ament_package()
