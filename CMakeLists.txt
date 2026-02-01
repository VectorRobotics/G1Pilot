cmake_minimum_required(VERSION 3.8)
project(g1_ik)

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
endif()

option(ROS2_INTEGRATION "Enable ROS 2 integration" ${ROS2_DEFAULT})

if(NOT ROS2_INTEGRATION)
    if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
        set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/install" CACHE PATH "Local install prefix" FORCE)
    endif()
endif()

set(CMAKE_INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party/lib)
include_directories(BEFORE SYSTEM ${CMAKE_CURRENT_SOURCE_DIR}/third_party/include)

# ---- Options & Dependencies ---- #

set(USE_CASADI ON CACHE BOOL "Use CasADi for optimization")
option(BUILD_TESTING "Build tests" OFF)
cmake_dependent_option(INSTALL_TESTING "Install test sources" OFF "BUILD_TESTING" ON)
cmake_dependent_option(RUN_TESTING "Run test scripts" BUILD_TESTING "NOT ROS2_INTEGRATION" OFF)

list(INSERT CMAKE_PREFIX_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/third_party")

if(USE_CASADI)
    add_definitions(-DPINOCCHIO_WITH_CASADI)
    find_package(casadi REQUIRED)
    message(STATUS "CasADi found: ${casadi_DIR}")

endif()

find_package(urdfdom REQUIRED)
find_package(console_bridge REQUIRED)
find_package(pinocchio REQUIRED)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen")

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

if(ROS2_INTEGRATION)
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