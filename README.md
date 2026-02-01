# G1_IK
  ![Alt text](docs/ik.gif)

`g1_ik` is a ROS2 package and also contains a C++ library for inverse kinematics and collision detection for the Unitree G1 HUmanoid robot.

## Key features
1. `arm_ik` C++ Library
   - Solving for inverse kinematics of the arms of the G1 give SE(3) or 6D pose for the end-effector.
   - Accounting for previous state for closest solution, naturalizing motion
   - Modelled as an optimization problem allowing tuning of weights of different costs (translation, rotation, regularization, and smothening) to achieve desired result.
   - Full inverse dynamics of the robot, assuming fixed legs, to generate joint level commands including gravity feedforward.
   - Optional build using the naive iteration method for inverse kinematics.
   - Self collision detection ensuring desired poses do not result in the arm colliding with itself.
   - Easy to freeze joints to simplify the IK problem as well as allow complex robot usage.
   - Full C++ implementation with simple API. 
2. `ik_joint_state_publisher`  ROS2 executable
   - Publishing joint states as received from the `arm_ik` library to the topic `ik/joint_states`
3. `display.launch.py` ROS2 launch file
   - Visualize the G1 robot in Rviz with TF tree
   - Launched joint_state_publisher, robot_state_publisher and rviz
   - Configures joint_state_publisher to use the `ik/joint_states` topic to publish full `/joint_states` for RViz visualization.

You don't need a robot to use this library.

# Installation

## Building with ROS2
Follow these instructions.
1. (Optional) Create a new workspace
   ```bash
   mkdir g1_ws g1_ws/src
   cd g1_ws/src
   ```
2. Clone the source code
   ```bash
   git clone https://github.com/VectorRobotics/g1_ik.git
   ```
3. Link third_party libraries to dynamic linker
   ```bash
   export LD_LIBRARY_PATH=$PWD/g1_ik/third_party/lib:$LD_LIBRARY_PATH
   ```
4. Build and install
   ```bash
   cd ..
   colcon build
   ```
5. Source your workspace installations
   ```bash
   source install/setup.bash
   ```

## Building C++ library from source
Follow these instructions.
1. Clone the source code

   ```bash
   git clone https://github.com/VectorRobotics/g1_ik.git
   ```
2. Link third_party libraries to dynamic linker
   ```bash
   export LD_LIBRARY_PATH=$PWD/g1_ik/third_party/lib:$LD_LIBRARY_PATH
   ```
3. Make build and install directories
   ```bash
   cd g1_ik && mkdir build install && cd build
   ```
4. Configure, make and install
   ```bash
   cmake ..
   make -j4
   make install
   ```



# How to use

## Use with ROS2
Run the launch file
```bash
ros2 launch g1_ik display.launch.py
```

If you just want to run the node
```bash
ros2 run g1_ik ik_joint_state_publisher
```

## Use as C++ library

To use IK, do the following in the package
```cpp
#include <arm_ik/robot_arm_ik_g1_23dof.h>

// Create robot configuration
RobotConfig config;
config.asset_file = "../assets/g1/g1_29dof_with_hand_rev_1_0.urdf";
config.asset_root = "../assets/g1/";

// Create IK solver with collision detection
G1_29_ArmIK_NoWrists arm_ik(false, false, &config);

// Define external forces (6D wrenches)
Eigen::VectorXd ext_force_left = Eigen::VectorXd::Zero(6);
Eigen::VectorXd ext_force_right = Eigen::VectorXd::Zero(6);
ext_force_left(2) = 5.0;  // 5N downward force

// Solve IK with collision checking
auto result = arm_ik.solve_ik(
    left_target, right_target,
    nullptr, nullptr,  // current q, dq
    &ext_force_left, &ext_force_right,
    true  // enable collision checking
);

// std::vector<string or double>
result.name
reuslt.position
result.velocity // Not implemented
result.effort
```

### Some helper functions
```cpp
// Helper function to create SE3 transformation matrix
Eigen::Matrix4d create_se3(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    transform.block<3, 1>(0, 3) = t;
    return transform;
}

// Helper function to create SE3 from quaternion components and translation
Eigen::Matrix4d create_se3(double qw, double qx, double qy, double qz, 
                           double tx, double ty, double tz) {
    Eigen::Quaterniond q(qw, qx, qy, qz);
    Eigen::Vector3d t(tx, ty, tz);
    return create_se3(q, t);
}
```



```bash
ln -s /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.83.0 /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.74.0
ln -s /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.83.0 /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.74.0
```