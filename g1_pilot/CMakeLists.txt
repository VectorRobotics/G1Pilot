cmake_minimum_required(VERSION 3.8)
project(g1_pilot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(CMakeDependentOption)

add_compile_options(-w -Wall -Wextra -Wpedantic)

# ---- Integration & Path Logic ---- #

if(DEFINED ENV{AMENT_PREFIX_PATH} OR DEFINED ENV{COLCON_PREFIX_PATH} OR DEFINED ENV{ROS_DISTRO})
    set(ROS2_DEFAULT ON)
    message(STATUS "Building with ROS2")
else()
    set(ROS2_DEFAULT OFF)
    message(WARNING "NOT building with ROS2")
    if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
        set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/install" CACHE PATH "Local install prefix" FORCE)
    endif()
endif()

list(APPEND CMAKE_PREFIX_PATH /opt/openrobots)
set(CMAKE_INSTALL_RPATH "/opt/openrobots/lib")
set(CMAKE_BUILD_RPATH "/opt/openrobots/lib")

# ---- Options & Dependencies ---- #

find_package(casadi REQUIRED)
find_package(pinocchio REQUIRED)
find_package(yaml-cpp REQUIRED)

option(BUILD_TESTING "Build tests" ON)
cmake_dependent_option(INSTALL_TESTING "Install test sources" OFF "BUILD_TESTING" ON)
cmake_dependent_option(RUN_TESTING "Run test scripts" BUILD_TESTING "NOT ROS2_DEFAULT" OFF)



# ---- Library ---- #

file(GLOB_RECURSE g1_pilot_SOURCES 
    "${CMAKE_CURRENT_SOURCE_DIR}/src/base/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/arm_ik/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/arm_control/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/arm_mp/*.cpp"
    
)
add_library(g1_pilot ${g1_pilot_SOURCES})

target_compile_definitions(g1_pilot PUBLIC COAL_DISABLE_HPP_FCL_WARNINGS)

target_include_directories(g1_pilot PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(g1_pilot PUBLIC Eigen3::Eigen pinocchio::pinocchio casadi::casadi yaml-cpp)

# ---- Testing ---- #

if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    
    add_executable(test_ik test/arm_ik/test_ik.cpp)
    target_link_libraries(test_ik PRIVATE g1_pilot GTest::gtest_main)

    add_executable(test_control test/arm_control/test_control.cpp)
    target_link_libraries(test_control PRIVATE g1_pilot GTest::gtest_main)

    add_executable(test_mp_ik test/arm_mp/test_traj_gen.cpp)
    target_link_libraries(test_mp_ik PRIVATE g1_pilot GTest::gtest_main)

    # add_executable(math_unittest test/arm_mp/math_unittest.cpp)
    # add_executable(line_unittest test/arm_mp/line_unittest.cpp)
    # target_link_libraries(math_unittest PRIVATE arm_mp GTest::GTest GTest::Main pthread)
    # target_link_libraries(line_unittest PRIVATE arm_mp GTest::GTest GTest::Main pthread)

    if (RUN_TESTING)
        include(GoogleTest)
        gtest_discover_tests(test_ik WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test)
        gtest_discover_tests(test_control WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test)
        # gtest_discover_tests(math_unittest WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test)
        # gtest_discover_tests(line_unittest WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test)
        gtest_discover_tests(test_mp_ik WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test)
    endif()


endif()

# --- Installation --- #

include(GNUInstallDirs)

install(TARGETS 
    g1_pilot 
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if(INSTALL_TESTING)
    install(FILES test/arm_ik/test_ik.cpp DESTINATION share/${PROJECT_NAME}/test)
    install(FILES test/arm_control/test_control.cpp DESTINATION share/${PROJECT_NAME}/test)
endif()

# ---- ROS 2 Integration ---- #

if(ROS2_DEFAULT)
    find_package(ament_cmake REQUIRED)

    set(ROS2_DEPS
        ament_index_cpp
        rclcpp
        rclcpp_action
        sensor_msgs
        nav_msgs
        geometry_msgs
        trajectory_msgs
        tf2_ros
        tf2_eigen
        tf2_geometry_msgs
        humanoid_manipulation_interfaces
    )

    foreach(dep ${ROS2_DEPS})
        find_package(${dep} REQUIRED)
    endforeach()

    add_executable(ik_joint_state_publisher src/joint_state_publisher.cpp)
    add_executable(impede_arms src/impede_arms.cpp)
    add_executable(joint_traj_publisher src/joint_traj_publisher.cpp)
    add_executable(visual_servo src/visual_servo.cpp)
    add_executable(grav_ff src/grav_ff.cpp)

    # Service Servers
    add_executable(plan_trajecory src/plan_trajecory.cpp)

    # Action Servers
    add_executable(controller src/controller.cpp)
    add_executable(joint_controller src/joint_controller.cpp)

    target_link_libraries(ik_joint_state_publisher g1_pilot)
    ament_target_dependencies(ik_joint_state_publisher ${ROS2_DEPS})

    target_link_libraries(impede_arms g1_pilot)
    ament_target_dependencies(impede_arms ${ROS2_DEPS})

    target_link_libraries(joint_traj_publisher g1_pilot)
    ament_target_dependencies(joint_traj_publisher ${ROS2_DEPS})

    target_link_libraries(visual_servo g1_pilot)
    ament_target_dependencies(visual_servo ${ROS2_DEPS})

    target_link_libraries(grav_ff g1_pilot)
    ament_target_dependencies(grav_ff ${ROS2_DEPS})

    target_link_libraries(controller g1_pilot)
    ament_target_dependencies(controller ${ROS2_DEPS})

    target_link_libraries(joint_controller g1_pilot)
    ament_target_dependencies(joint_controller ${ROS2_DEPS})

    target_link_libraries(plan_trajecory g1_pilot)
    ament_target_dependencies(plan_trajecory ${ROS2_DEPS})

    install(TARGETS
        ik_joint_state_publisher
        impede_arms
        joint_traj_publisher
        visual_servo
        grav_ff
        controller
        plan_trajecory
        joint_controller
        DESTINATION lib/${PROJECT_NAME}
    )

    install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
    install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

    if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
    endif()

    ament_package()
endif()