# Installation
## Building from C++ library from source
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