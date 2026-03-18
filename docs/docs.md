# G1Pilot - Technical Documentation

![Inverse Kinematics Demo](ik.gif)

`G1Pilot` is a C++ library and ROS 2 package for controlling the upper body of the **Unitree G1 humanoid robot**. It provides inverse kinematics, impedance control, polynomial trajectory generation, visual servoing, self-collision detection, and gravity feedforward — all built on top of [Pinocchio](https://stack-of-tasks.github.io/pinocchio/) and [CasADi](https://web.casadi.org/).

No robot hardware is required to use the library.

---

## Table of Contents

- [Architecture Overview](#architecture-overview)
- [Submodules](#submodules)
  - [base — Robot Model & Utilities](#base--robot-model--utilities)
  - [arm_ik — Inverse Kinematics](#arm_ik--inverse-kinematics)
  - [arm_control — Controllers](#arm_control--controllers)
  - [arm_mp — Motion Planning](#arm_mp--motion-planning)
- [ROS 2 Nodes](#ros-2-nodes)
- [Design Decisions](#design-decisions)
- [Usage Guide](#usage-guide)
  - [As a C++ Library](#as-a-c-library)
  - [With ROS 2](#with-ros-2)
- [ROS 2 Topic Interface](#ros-2-topic-interface)
- [Configuration & Parameters](#configuration--parameters)
- [Testing](#testing)

---

## Architecture Overview

The package is split into a **standalone C++ library** (`libg1_pilot`) and **ROS 2 executables** that consume it. The library compiles and installs independently of ROS 2 — the CMake build detects whether it is inside a colcon/ament workspace and conditionally enables ROS integration.

```
G1Pilot/
├── include/                    # Public headers
│   ├── g1_pilot/g1_pilot.h     # Top-level umbrella header
│   ├── base/                   # Robot model, interfaces
│   ├── arm_ik/                 # IK solver
│   ├── arm_control/            # Controllers
│   └── arm_mp/                 # Motion planning
├── src/
│   ├── base/                   # Library: robot model
│   ├── arm_ik/                 # Library: IK solver
│   ├── arm_control/            # Library: controllers
│   ├── arm_mp/                 # Library: motion planner
│   ├── visual_servo.cpp        # ROS node: full visual servoing pipeline
│   ├── visual_servo_ik.cpp     # ROS node: IK-only visual servoing
│   ├── impede_arms.cpp         # ROS node: impedance control
│   ├── grav_ff.cpp             # ROS node: gravity feedforward only
│   ├── joint_traj_publisher.cpp# ROS node: trajectory demo
│   ├── joint_state_publisher.cpp# ROS node: IK demo
│   └── helper_funcs.h          # Shared ROS-node utilities
├── launch/                     # ROS 2 launch files
├── assets/g1/                  # URDF + collision meshes (178 STL files)
├── rviz/                       # RViz configurations
└── test/                       # Google Test suites
```

### Dependency Graph

```
g1_pilot.h  (umbrella)
    │
    ├── base/g1.h            ─── G1DualArm (entry point)
    │       │
    │       └── base/interfaces.h  ─── JointState, RobotConfig
    │
    ├── arm_ik/arm_ik.h
    │       └── robot_arm_ik.h     ─── HumanoidIK (CasADi/IPOPT)
    │               ├── utils.h                (Eigen ↔ CasADi)
    │               └── weighted_moving_filter.h
    │
    ├── arm_control/arm_control.h
    │       ├── impedance_control.h  ─── ImpedanceController
    │       └── ik_traj_tracker.h    ─── IKTrajTracker
    │               └── base_controller.h ─── ArmController (base class)
    │
    └── arm_mp/arm_mp.h
            └── visual_servo_planner.h ─── VisualServoPlanner
                    └── poly_traj_gen.h ─── PolynomialTrajectoryGenerator
```

Everything lives in the `ArmPilot` namespace.

---

## Submodules

### base — Robot Model & Utilities

**Files:** `include/base/g1.h`, `include/base/interfaces.h`, `src/base/g1.cpp`

#### `G1DualArm`

The central entry point. Constructing a `G1DualArm` performs the full initialization chain:

1. **Loads the URDF** (29-DOF G1 model) via Pinocchio, including collision geometries from STL meshes.
2. **Categorizes joints** into semantic groups — left/right legs, waist, upper arms, wrists, hands.
3. **Adds end-effector frames** (`L_ee`, `R_ee`) at the fingertip of the index finger, offset from the wrist yaw joint.
4. **Builds reduced models** by freezing joints that are irrelevant for arm control:
   - `model_g1_26` — waist joints (roll, pitch) frozen
   - `model_g1_14` — legs frozen (the robot is assumed standing)
   - `model_g1_14_wo_hands` — finger joints frozen
5. **Instantiates the tool chain:**
   - `HumanoidIK* ik` — for inverse kinematics
   - `IKTrajTracker* controller` — for trajectory tracking
   - `VisualServoPlanner* motion_planner` — for trajectory generation

The 14-DOF model without hands (`model_g1_14_wo_hands`) is the one passed to the IK solver and controller — this keeps the optimization tractable while covering all arm DOFs (7 per arm: 4 upper arm + 3 wrist).

#### Joint Categorization (29 DOF)

| Group | Joints | Count |
|-------|--------|-------|
| Left leg | hip (pitch/roll/yaw), knee, ankle (pitch/roll) | 6 |
| Right leg | hip (pitch/roll/yaw), knee, ankle (pitch/roll) | 6 |
| Waist | yaw, roll, pitch | 3 |
| Left upper arm | shoulder (pitch/roll/yaw), elbow | 4 |
| Right upper arm | shoulder (pitch/roll/yaw), elbow | 4 |
| Left wrist | pitch, roll, yaw | 3 |
| Right wrist | pitch, roll, yaw | 3 |

*(Waist yaw is kept in the reduced model but locked to near-zero via joint limits in the optimization.)*

#### Data Structures

```cpp
struct JointState {
    std::vector<std::string> name;
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> effort;
};

struct RobotConfig {
    std::string asset_file;   // path to URDF
    std::string asset_root;   // path to mesh directory
    int NUM_DOF = 29;
};
```

Utility functions `jointstate_to_vectors()` and `vectors_to_jointstate()` convert between `JointState` and Pinocchio-native `Eigen::VectorXd` representations, using the model's joint indexing.

---

### arm_ik — Inverse Kinematics

**Files:** `include/arm_ik/robot_arm_ik.h`, `src/arm_ik/robot_arm_ik.cpp`, plus `utils.h` and `weighted_moving_filter.h`

#### `HumanoidIK`

Solves inverse kinematics as a **nonlinear optimization** using CasADi + IPOPT.

##### How It Works

1. **Symbolic model**: The Pinocchio model is cast to `casadi::SX` symbolic type. Forward kinematics is evaluated symbolically to express end-effector poses as functions of joint angles.

2. **Cost function**: A weighted sum of four terms:

   | Term | Weight | Purpose |
   |------|--------|---------|
   | Translation error (L2) | 50.0 | Reach the target position |
   | Rotation error (L2) | 3.0 | Match target orientation |
   | Joint regularization | 0.02 | Prefer solutions near zero configuration |
   | Smoothness (q - q_last) | 0.1 | Penalize large jumps between consecutive solves |

   The SE(3) error is computed via `log6(T_current.actInv(T_target))`, the Lie-algebraic error.

3. **Three separate optimizers**: `opti_` (both arms), `opti_l_` (left only), `opti_r_` (right only). Each has its own CasADi `Opti` problem, variables, and parameters.

4. **Joint limits**: Enforced as box constraints. Waist joints are locked to ±0.0001 rad.

5. **IPOPT settings**: Max 50 iterations, tolerance 1e-6, warm-start enabled for real-time performance.

##### Self-Collision Detection

On initialization, all collision pairs from the geometry model are registered. Adjacent links (parent-child in the kinematic tree) are filtered out to avoid spurious collisions between connected bodies.

##### Solution Smoothing

A `WeightedMovingFilter` with weights `[0.4, 0.3, 0.2, 0.1]` (window size 4) smooths the IK output, reducing jitter at the cost of slight lag.

##### External Wrench Support

After solving IK, the solver can compute feedforward joint torques that compensate for external wrenches applied at the end-effectors (e.g., contact forces). This uses the frame Jacobian transpose method: `tau = J^T * F_ext`.

##### Fallback (Non-CasADi Build)

When built without CasADi (`BUILD_WITH_CASADI_SUPPORT=OFF`), a simpler iterative Jacobian-based IK is available (damped least-squares with `DT=0.1`, `damp=1e-6`, up to 1000 iterations).

#### API

```cpp
// Both arms simultaneously
JointState solve_ik(
    const Eigen::Matrix4d& left_wrist,
    const Eigen::Matrix4d& right_wrist,
    const Eigen::VectorXd* current_q = nullptr,
    const Eigen::VectorXd* current_dq = nullptr,
    const Eigen::VectorXd* EE_efrc_L = nullptr,
    const Eigen::VectorXd* EE_efrc_R = nullptr
);

// Single arm (left=true for left arm)
JointState solve_ik(
    const Eigen::Matrix4d& wrist,
    const bool left = false,
    const Eigen::VectorXd* current_q = nullptr,
    const Eigen::VectorXd* current_dq = nullptr,
    const Eigen::VectorXd* EE_efrc = nullptr
);
```

---

### arm_control — Controllers

**Files:** `include/arm_control/`, `src/arm_control/`

All controllers inherit from `ArmController`, which provides:
- Forward kinematics update (joint positions → end-effector poses, Jacobians, velocities)
- Gravity compensation via `pinocchio::computeGeneralizedGravity()`
- A `control_no_arms()` method that publishes only gravity feedforward

#### `ImpedanceController`

Task-space impedance control in the end-effector body frame.

**Control law:**

```
tau = J_body^T * (Kp * e_twist + Kd * e_vel + Ki * integral(e_twist))
    + gravity_compensation
```

where:
- `e_twist = log6(T_current^{-1} * T_desired)` — SE(3) pose error in body frame
- `e_vel = v_desired - v_current` — velocity error in body frame
- Anti-windup clamp at ±5.0 on the integral term

Default gains: `Kp_linear=30`, `Kp_angular=5`, `Kd/Ki=0`.

Supports: `control_both_arms()`, `control_left_arm()`, `control_right_arm()`.

#### `IKTrajTracker`

An alternative controller that delegates to the IK solver at each time step rather than computing torques directly. Given a desired end-effector pose, it:

1. Calls `HumanoidIK::solve_ik()` to get joint-level position + effort commands
2. Computes an error metric combining linear and angular terms

This is used by the visual servoing pipeline where trajectory waypoints are tracked sequentially.

---

### arm_mp — Motion Planning

**Files:** `include/arm_mp/`, `src/arm_mp/`

#### `PolynomialTrajectoryGenerator`

Generates smooth trajectories on SE(3) using **polynomial interpolation in the Lie algebra**.

**Method:**

1. Compute the relative transform: `T_rel = T_start^{-1} * T_goal`
2. Take the matrix logarithm: `xi = log(T_rel)` — a 4x4 element of se(3)
3. Extract the 6D twist via the `vee` map: `[v; omega]`
4. Fit a polynomial (default order 5) to interpolate from `[0,0,0,0,0,0]` to the twist, with boundary conditions on velocity and acceleration
5. Evaluate the polynomial at uniform time steps
6. Map each twist back to SE(3) via `T(s) = T_start * exp(hat(xi(s)))` — the `hat` and matrix exponential maps

**Time scaling:** Duration is computed from translational distance divided by `MAX_LIN_VEL` (default 0.1 m/s). Velocities and accelerations are scaled to the `[0,1]` parameter domain.

**Constraint system:** The 5th-order polynomial satisfies 6 boundary conditions — position, velocity, and acceleration at both start and goal. The constraint matrix is pre-computed and pseudo-inverted at construction time for efficiency.

#### `VisualServoPlanner`

Extends `PolynomialTrajectoryGenerator` for visual servoing with a **two-phase approach strategy**:

```
Phase 1: Move from current pose to an intermediate pose
         (offset along the -x axis of the target by 5cm)

Phase 2: Move from intermediate pose straight into the target
         (along the target's x-axis)
```

This ensures the end-effector approaches the target **head-on** along the contact normal (x-axis), which is critical for manipulation tasks where the approach direction matters (e.g., pressing a button, grasping).

When the end-effector is already close (< 10cm), the intermediate phase is skipped and a direct trajectory is generated.

The final approach leg is pre-computed at construction time (20 steps) since it is always the same relative motion.

---

## ROS 2 Nodes

### `visual_servo` — Full Visual Servoing Pipeline

The main application node. Combines motion planning, trajectory tracking, and real-time control.

**Workflow:**
1. Subscribes to `/goal_pose` (PoseStamped) for target end-effector poses
2. Validates the goal against workspace limits (position: `[0.1, -0.6, 0.0]` to `[0.6, 0.6, 1.0]`, orientation: roll < 90deg, pitch/yaw < 45deg)
3. Plans a trajectory using `VisualServoPlanner::planTrajectory()`
4. Publishes the trajectory on `/traj` (Path) for visualization
5. At 20 Hz, tracks the trajectory with `IKTrajTracker`:
   - Pops the next waypoint when end-effector error < 2cm
   - Publishes joint commands to `position_control`
6. When idle (no trajectory), publishes gravity compensation only

**Topics:**
| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Sub | `/goal_pose` | `PoseStamped` | Target EE pose in pelvis frame |
| Sub | `/feedback` | `JointState` | Current joint state from robot |
| Pub | `position_control` | `JointState` | Joint position commands |
| Pub | `traj` | `Path` | Planned trajectory for RViz |

### `visual_servo_ik` — IK-Only Goal Tracking

Simpler node that solves IK directly for the goal pose (no trajectory planning or impedance control). Publishes position commands at 10 Hz.

**Topics:**
| Direction | Topic | Type |
|-----------|-------|------|
| Sub | `/goal_pose` | `PoseStamped` |
| Pub | `position_control` | `JointState` |

### `impede_arms` — Impedance Control

Subscribes to feedback joint states and computes impedance control torques. Accepts goal poses via `/goal_pose`.

**Topics:**
| Direction | Topic | Type |
|-----------|-------|------|
| Sub | `feedback` | `JointState` |
| Sub | `/goal_pose` | `PoseStamped` |
| Pub | `effort_control` | `JointState` |

### `grav_ff` — Gravity Feedforward

Minimal node. Reads current joint state, computes gravity compensation torques using the full inverse dynamics (RNEA), and publishes them.

**Topics:**
| Direction | Topic | Type |
|-----------|-------|------|
| Sub | `feedback` | `JointState` |
| Pub | `effort_control` | `JointState` |

### `joint_traj_publisher` — Trajectory Demo

Demonstrates motion planning by commanding the left arm to move forward and up. Plans a trajectory every 3 seconds and publishes IK solutions for each waypoint.

### `ik_joint_state_publisher` — IK Demo

Publishes IK solutions at 10 Hz for a fixed target pose. Waits for a `/goal_pose` before starting.

---

## Design Decisions

### Model Reduction Strategy

Rather than solving IK for all 29 DOFs, the model is progressively reduced:
- **Freeze waist** (roll, pitch) → standing posture assumed
- **Freeze legs** → the robot is stationary
- **Freeze hand joints** → fingers are not controlled by the IK solver

This reduces the optimization from 29 variables to 14 (7 per arm), making real-time IK feasible with IPOPT.

Waist yaw is kept in the model but effectively locked via tight joint limits (±0.0001 rad) in the optimizer — this preserves the kinematic chain structure while preventing waist rotation.

### Lie Group Trajectory Generation

Trajectories are generated on SE(3) via the exponential map rather than interpolating position and orientation separately. This avoids gimbal lock, ensures valid rotation matrices at every point, and produces geodesic (shortest-path) motions in the group.

### Dual Build System

The CMake build supports two modes:
- **With ROS 2**: Detected via `AMENT_PREFIX_PATH` / `COLCON_PREFIX_PATH` / `ROS_DISTRO`. Builds the library + all ROS nodes.
- **Standalone C++**: Builds only the library, installed to a local `install/` directory. No ROS dependencies needed.

### CasADi vs Iterative IK

The CasADi/IPOPT approach handles joint limits natively as box constraints and produces globally smoother solutions. The iterative fallback (compiled with `BUILD_WITH_CASADI_SUPPORT=OFF`) is lighter-weight but has no constraint handling beyond clamping.

### Error Metric

Both controllers use a combined error metric:
```
error = sqrt(||e_linear||^2 + 0.05 * ||e_angular||^2)
```
The 0.05 weight on angular error prioritizes position accuracy over orientation, which is appropriate for reach-and-touch tasks.

---

## Usage Guide

### As a C++ Library

After installing the library (see README for build instructions), include the umbrella header:

```cpp
#include <g1_pilot/g1_pilot.h>
using namespace ArmPilot;
```

#### Initialize the robot

```cpp
RobotConfig config;
config.asset_file = "/path/to/g1_29dof_with_hand_rev_1_0.urdf";
config.asset_root = "/path/to/g1/meshes/";
config.NUM_DOF = 29;

G1DualArm arm(&config);
```

#### Solve IK for both arms

```cpp
Eigen::Matrix4d left_target = Eigen::Matrix4d::Identity();
left_target.block<3,1>(0,3) = Eigen::Vector3d(0.2, 0.2, 0.1);

Eigen::Matrix4d right_target = Eigen::Matrix4d::Identity();
right_target.block<3,1>(0,3) = Eigen::Vector3d(0.2, -0.2, 0.1);

JointState result = arm.ik->solve_ik(left_target, right_target);
// result.name     — joint names
// result.position — joint angles (rad)
// result.effort   — gravity feedforward torques (Nm)
```

#### Solve IK for a single arm

```cpp
JointState result = arm.ik->solve_ik(right_target, false);  // right arm
JointState result = arm.ik->solve_ik(left_target, true);    // left arm
```

#### Solve IK with current state (warm-start)

```cpp
Eigen::VectorXd current_q = /* from encoder readings */;
JointState result = arm.ik->solve_ik(
    left_target, right_target,
    &current_q, nullptr  // pass current q for warm-starting
);
```

#### Solve IK with external forces

```cpp
Eigen::VectorXd force_right = Eigen::VectorXd::Zero(6);
force_right(2) = 5.0;  // 5N downward at right end-effector

JointState result = arm.ik->solve_ik(
    left_target, right_target,
    nullptr, nullptr,
    nullptr, &force_right
);
// result.effort now includes compensation for the external force
```

#### Generate a trajectory

```cpp
Eigen::Matrix4d start = Eigen::Matrix4d::Identity();
start.block<3,1>(0,3) = Eigen::Vector3d(0.2, -0.15, 0.1);

Eigen::Matrix4d goal = Eigen::Matrix4d::Identity();
goal.block<3,1>(0,3) = Eigen::Vector3d(0.35, -0.15, 0.25);

std::vector<Eigen::MatrixXd> trajectory = arm.motion_planner->planTrajectory(
    &goal,     // goal pose (4x4)
    &start     // start pose (4x4)
);

// trajectory is a vector of 4x4 SE(3) waypoints
for (auto& pose : trajectory) {
    JointState ik_result = arm.ik->solve_ik(pose, false);
    // send ik_result.position to actuators...
}
```

#### Gravity feedforward only

```cpp
JointState current_state;
current_state.name = {"waist_yaw_joint", "left_shoulder_pitch_joint", ...};
current_state.position = {0.0, 0.1, ...};
current_state.velocity = {0.0, 0.0, ...};
current_state.effort = {0.0, 0.0, ...};

JointState grav = arm.grav_ff(current_state);
// grav.effort contains the gravity compensation torques
```

#### Helper: create SE(3) from quaternion + translation

```cpp
#include "helper_funcs.h"  // only available in ROS node source

// From quaternion components (w, x, y, z) and translation (x, y, z)
Eigen::Matrix4d T = create_se3(1.0, 0.0, 0.0, 0.0,  // qw, qx, qy, qz
                                0.2, -0.15, 0.1);      // tx, ty, tz

// From Eigen types
Eigen::Quaterniond q(1.0, 0.0, 0.0, 0.0);
Eigen::Vector3d t(0.2, -0.15, 0.1);
Eigen::Matrix4d T = create_se3(q, t);
```

### With ROS 2

#### Launch the display (visualization only)

```bash
ros2 launch g1_pilot display.launch.py
```

Opens RViz with the G1 robot model. Joint states are read from `/feedback`.

#### Run the IK demo

```bash
ros2 launch g1_pilot ik_demo.launch.py
```

Then publish a goal pose:
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'pelvis'}, pose: {position: {x: 0.3, y: -0.15, z: 0.2}, orientation: {w: 1.0}}}"
```

#### Run visual servoing

```bash
ros2 launch g1_pilot visual_servo.launch.py
```

The node will wait for `/feedback` joint states and `/goal_pose` targets. Send a goal:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'pelvis'}, pose: {position: {x: 0.4, y: -0.15, z: 0.2}, orientation: {w: 1.0}}}"
```

The planned trajectory appears in RViz on the `/traj` topic.

#### Run visual servoing with ArUco detection

```bash
ros2 launch g1_pilot visual_servo_aruco.launch.py
```

This additionally launches a RealSense camera and ArUco marker detector (requires `realsense2_camera` and `hand_eye_calibration` packages).

#### Run impedance control

```bash
ros2 launch g1_pilot impede_arms_demo.launch.py
```

Requires `/feedback` joint states from the robot. Publishes torque commands on `effort_control`.

#### Run gravity feedforward

```bash
ros2 launch g1_pilot grav_ff.launch.py
```

Subscribes to `feedback`, publishes gravity compensation torques on `effort_control`.

#### Run trajectory demo

```bash
ros2 launch g1_pilot joint_traj_demo.launch.py
```

Automatically generates trajectories every 3 seconds and visualizes them.

---

## ROS 2 Topic Interface

### Input Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/feedback` | `sensor_msgs/JointState` | Current robot joint state (from hardware driver) |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Desired end-effector pose in pelvis frame |

### Output Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `position_control` | `sensor_msgs/JointState` | Joint position commands (IK/trajectory nodes) |
| `effort_control` | `sensor_msgs/JointState` | Joint torque commands (impedance/gravity nodes) |
| `traj` | `nav_msgs/Path` | Planned trajectory for RViz visualization |

---

## Configuration & Parameters

### ROS Parameters

All nodes accept:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `asset_file` | `<share>/assets/g1/g1_29dof_with_hand_rev_1_0.urdf` | Path to URDF |
| `asset_root` | `<share>/assets/g1/` | Path to mesh directory |
| `num_dof` | `29` | Number of DOFs in the URDF |

### Workspace Limits

Goal poses are validated against these bounds (in the pelvis frame):

| Axis | Min | Max |
|------|-----|-----|
| X | 0.1 m | 0.6 m |
| Y | -0.6 m | 0.6 m |
| Z | 0.0 m | 1.0 m |
| Roll | | ±90 deg |
| Pitch | | ±45 deg |
| Yaw | | ±45 deg |

### Controller Gains

Gains are set programmatically. Typical values used in the ROS nodes:

**IK Trajectory Tracker (visual_servo):**
- `Kp_linear = 100`, `Kp_angular = 1`
- `Kd_linear = 2`, `Kd_angular = 0.2`

**Impedance Controller (impede_arms):**
- `Kp_linear = 100`, `Kp_angular = 1`
- `Kd_linear = 2`, `Kd_angular = 0.2`

### Motion Planner Defaults

| Parameter | Default |
|-----------|---------|
| Polynomial order | 5 |
| MAX_LIN_VEL | 0.1 m/s |
| MAX_ANG_VEL | 4.0 rad/s |
| MAX_LIN_ACC | 0.05 m/s^2 |
| MAX_ANG_ACC | 4.0 rad/s^2 |
| Time step | 0.05 s |
| Visual servo offset | 0.05 m |

---

## Testing

The package includes Google Test suites:

```bash
# Build with testing enabled (default)
colcon build

# Run all tests
cd build/g1_pilot
ctest

# Or run individually
./test_ik           # IK solver tests
./test_control      # Controller tests
./test_mp_ik        # Motion planning + IK integration tests
```

### Test Coverage

| Test | What it verifies |
|------|-----------------|
| `test_ik` | Dual-arm IK solve, timing, CasADi integration |
| `test_control` | G1DualArm initialization, controller setup |
| `test_mp_ik` | Trajectory generation, IK solution along trajectory |
| `math_unittest` | Polynomial math: power vectors, differentiation matrices, evaluation |
| `test_joint_traj` | Boundary conditions (start/goal position, velocity, acceleration) |

A Python visualization script (`test/arm_mp/line_viz.py`) plots 3D trajectories exported as CSV from the tests.
