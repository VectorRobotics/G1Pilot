cmake_minimum_required(VERSION 3.8)
project(g1_pilot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(CMakeDependentOption)

add_compile_options(-w -Wall -Wextra -Wpedantic)

# ---- Integration & Path Logic ---- #

if(DEFINED ENV{COLCON_PREFIX_PATH})
    set(ROS2_DEFAULT ON)
else()
    set(ROS2_DEFAULT OFF)
    if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
        set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/install" CACHE PATH "Local install prefix" FORCE)
    endif()
endif()

list(APPEND CMAKE_PREFIX_PATH /opt/openrobots)


# ---- Options & Dependencies ---- #
set(COAL_DISABLE_HPP_FCL_WARNINGS TRUE)
option(BUILD_WITH_CASADI_SUPPORT "Build the library with the support of CASADI" ON)
if(BUILD_WITH_CASADI_SUPPORT)
    set(USE_CASADI ON)
    find_package(casadi REQUIRED)
else()
    set (USE_CASADI OFF)
endif()

find_package(pinocchio REQUIRED)

option(BUILD_TESTING "Build tests" OFF)
cmake_dependent_option(INSTALL_TESTING "Install test sources" OFF "BUILD_TESTING" ON)
cmake_dependent_option(RUN_TESTING "Run test scripts" BUILD_TESTING "NOT ROS2_DEFAULT" OFF)



# ---- Library ---- #

file(GLOB_RECURSE g1_pilot_SOURCES 
    "${CMAKE_CURRENT_SOURCE_DIR}/src/base/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/arm_ik/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/arm_control/*.cpp"
    # "${CMAKE_CURRENT_SOURCE_DIR}/src/arm_mp/*.cpp"
    
)
add_library(g1_pilot ${g1_pilot_SOURCES})

target_include_directories(g1_pilot PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(g1_pilot PUBLIC Eigen3::Eigen pinocchio::pinocchio)

if(USE_CASADI)
    target_link_libraries(g1_pilot PUBLIC casadi)
    target_compile_definitions(g1_pilot PRIVATE USE_CASADI)
endif()

# ---- Testing ---- #

if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    
    add_executable(test_ik test/arm_ik/test_ik.cpp)
    add_executable(test_control test/arm_control/test_control.cpp)
    target_link_libraries(test_ik PRIVATE g1_pilot GTest::gtest_main)
    target_link_libraries(test_control PRIVATE g1_pilot GTest::gtest_main)

    if(USE_CASADI)
        target_compile_definitions(test_ik PRIVATE USE_CASADI)
    endif()

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
install(DIRECTORY assets/  DESTINATION share/${PROJECT_NAME}/assets)

if(INSTALL_TESTING)
    install(FILES test/arm_ik/test_ik.cpp DESTINATION share/${PROJECT_NAME}/test)
    install(FILES test/arm_control/test_control.cpp DESTINATION share/${PROJECT_NAME}/test)
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/assets/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/assets)
endif()

# ---- ROS 2 Integration ---- #

if(ROS2_DEFAULT)
    find_package(ament_cmake REQUIRED)
    find_package(ament_index_cpp REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)

    add_executable(ik_joint_state_publisher src/joint_state_publisher.cpp)
    target_link_libraries(ik_joint_state_publisher g1_pilot)
    ament_target_dependencies(ik_joint_state_publisher ament_index_cpp rclcpp sensor_msgs)

    install (TARGETS ik_joint_state_publisher
        DESTINATION lib/${PROJECT_NAME}
    )

    install(DIRECTORY launch assets rviz DESTINATION share/${PROJECT_NAME})

    if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
    endif()

    ament_package()
endif()