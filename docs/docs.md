# G1Pilot — Technical Documentation

![Inverse Kinematics Demo](ik.gif)

`G1Pilot` is the upper-body manipulation stack for the **Unitree G1 humanoid**. The repo is a single colcon workspace overlay containing four packages: a C++ library + ROS 2 nodes for kinematics/control/planning, a robot description package, a Python skills package for high-level behaviours, and a custom interfaces package for actions/services.

The core C++ library compiles standalone (no ROS 2) and provides inverse kinematics, impedance control, polynomial trajectory generation, OMPL-based joint-space planning, visual servoing approach planning, self-collision detection, and gravity feedforward — all built on top of [Pinocchio](https://stack-of-tasks.github.io/pinocchio/), [CasADi](https://web.casadi.org/), and [OMPL](https://ompl.kavrakilab.org/).

No robot hardware is required.

---

## Table of Contents

- [Repository Layout](#repository-layout)
- [Architecture Overview](#architecture-overview)
- [Configuration File (`g1.yaml`)](#configuration-file-g1yaml)
- [Library Submodules](#library-submodules)
  - [base — Robot Model & Utilities](#base--robot-model--utilities)
  - [arm_ik — Inverse Kinematics](#arm_ik--inverse-kinematics)
  - [arm_control — Controllers](#arm_control--controllers)
  - [arm_mp — Motion Planning](#arm_mp--motion-planning)
- [g1_pilot ROS 2 Nodes](#g1_pilot-ros-2-nodes)
- [humanoid_arm_skills (Python)](#humanoid_arm_skills-python)
- [humanoid_manipulation_interfaces](#humanoid_manipulation_interfaces)
- [g1_description](#g1_description)
- [Design Decisions](#design-decisions)
- [Usage Guide](#usage-guide)
- [ROS 2 Topic / Service / Action Interface](#ros-2-topic--service--action-interface)
- [Configuration & Parameters](#configuration--parameters)
- [Testing](#testing)

---

## Repository Layout

```
g1_pilot/                                 # workspace overlay (colcon src/)
├── g1_pilot/                             # core library + ROS 2 executables (C++)
├── g1_description/                       # URDFs, meshes, RViz config, display launcher
├── humanoid_arm_skills/                  # Python: visual-servo action server, pose filter
├── humanoid_manipulation_interfaces/     # action + srv definitions
├── docs/                                 # this file + assets
├── download_dependencies.sh              # robotpkg + Pinocchio install script
└── README.md
```

| Package | Type | Purpose |
|---------|------|---------|
| `g1_pilot` | `ament_cmake` | C++ library `libg1_pilot` + 8 ROS 2 executables (controllers, planners, demos, action/service servers). |
| `g1_description` | `ament_cmake` | URDFs (3 variants), 160 STL meshes, `display.launch.py` (RViz + dual `robot_state_publisher`s for feedback vs. control visualisation). |
| `humanoid_arm_skills` | `ament_python` | High-level skill nodes — visual servoing action server, pose filter, noisy-pose publisher for testing. |
| `humanoid_manipulation_interfaces` | `ament_cmake` | 3 action types, 2 service types — the contract between skills and controllers. |

---

## Architecture Overview

The library is split into a **standalone C++ library** (`libg1_pilot`) and **ROS 2 executables** that consume it. The CMake build detects whether it is inside a colcon workspace (via `AMENT_PREFIX_PATH` / `COLCON_PREFIX_PATH` / `ROS_DISTRO`) and conditionally enables ROS integration; outside a workspace it installs to a local `install/` prefix.

```
g1_pilot/g1_pilot/
├── include/                       # public headers
│   ├── g1_pilot/g1_pilot.h        # umbrella header
│   ├── base/                      # Humanoid class, interfaces
│   ├── arm_ik/                    # IK solver
│   ├── arm_control/               # controllers (impedance, IK trajectory tracker)
│   └── arm_mp/                    # motion planners (OMPL, polynomial, visual servo)
├── src/
│   ├── base/                      # library: humanoid model load, joint reduction, EE frames
│   ├── arm_ik/                    # library: CasADi IK
│   ├── arm_control/               # library: impedance + IK tracker
│   ├── arm_mp/                    # library: poly/joint-space/visual-servo planners
│   ├── joint_state_publisher.cpp  # node: ik_joint_state_publisher (IK demo)
│   ├── joint_traj_publisher.cpp   # node: joint_traj_publisher (trajectory demo)
│   ├── visual_servo.cpp           # node: visual_servo (full pipeline, dual-arm)
│   ├── impede_arms.cpp            # node: impede_arms (impedance control)
│   ├── grav_ff.cpp                # node: grav_ff (gravity feedforward only)
│   ├── plan_trajecory.cpp         # node: plan_trajecory (PlanJointTrajectory service server)
│   ├── controller.cpp             # node: controller (ControlTrajectory action server, SE(3))
│   ├── joint_controller.cpp       # node: joint_controller (ControlJointTrajectory action server)
│   └── helper_funcs.h             # ROS-side helpers: SE(3) builders, msg conversions, workspace check
├── launch/                        # 8 ROS 2 launch files
├── config/g1.yaml                 # joint groups, lock profiles, EE frame definitions
└── test/                          # Google Test suites
```

### Dependency Graph

```
g1_pilot.h  (umbrella)
    │
    ├── base/humanoid.h         ─── Humanoid (entry point; loads URDF + g1.yaml)
    │       │
    │       └── base/interfaces.h   ─── JointState, RobotConfig
    │
    ├── arm_ik/arm_ik.h
    │       └── robot_arm_ik.h      ─── HumanoidIK (CasADi/IPOPT)
    │               ├── utils.h                 (Eigen ↔ CasADi)
    │               └── weighted_moving_filter.h
    │
    ├── arm_control/arm_control.h
    │       ├── base_controller.h   ─── ArmController (FK + grav comp + control_no_arms)
    │       ├── impedance_control.h ─── ImpedanceController (with nullspace + rate limit)
    │       └── ik_traj_tracker.h   ─── IKTrajTracker
    │
    └── arm_mp/arm_mp.h
            ├── poly_traj_gen.h          ─── PolynomialTrajectoryGenerator (SE(3) polynomial)
            ├── joint_space_planner.h    ─── JointSpacePlanner (OMPL RRT*)  ◄─ default
            └── visual_servo_planner.h   ─── VisualServoPlanner (OMPL + linear final approach)
```

Everything lives in the **`HumanoidPilot`** namespace.

> Note: a `Humanoid` instance instantiates `HumanoidIK`, `IKTrajTracker`, and a `JointSpacePlanner` by default. The polynomial / visual-servo planners are still part of the library and can be constructed directly by callers, but they are **not** owned by `Humanoid` in the current build.

---

## Configuration File (`g1.yaml`)

`g1_pilot/g1_pilot/config/g1.yaml` is parsed at construction time by `Humanoid` and selects which joints participate in IK/control. The file replaces the previously hard-coded joint list in C++.

Top-level sections:

| Section | Purpose |
|---------|---------|
| `root_link` | URDF link used as kinematic root for all returned frames (default: `pelvis`). |
| `joint_categories` | Named groups of URDF joint names: `left_leg`, `right_leg`, `waist`, `left_upper_arm`, `right_upper_arm`, `left_wrist`, `right_wrist`, `left_gripper`, `right_gripper`. Reference only — they describe the URDF, not behaviour. |
| `configs` | Named profiles. Each profile picks `lock_categories`, optional `lock_joints`, optional `joint_limit_overrides` (per-joint `lower`/`upper`), and `end_effectors` (per-arm `parent_joint` + `offset`). |
| `active_config` | The currently active profile. |

Two profiles ship by default:

| Profile | Locked | Free DOF | Notes |
|---------|--------|----------|-------|
| `arms_with_waist` *(active)* | both legs + both grippers | 11 (2×7 arm + 3 waist; waist tightly clamped via overrides) | Joint-limit overrides shrink shoulder/wrist ranges and pin all 3 waist joints to ±1e-4 rad. |
| `arms_locked_waist` | both legs + both grippers + waist | 8 (2×7 arm) | Lighter optimisation when the waist is mechanically fixed. |

The end-effector frame is attached to the parent joint with a translation offset (default `[0.2108, ±0.0046, 0.0285]` m from `wrist_yaw_joint` — i.e. fingertip of the index finger).

**To change the active profile**, edit `active_config:` and rebuild — no code changes needed.

---

## Library Submodules

### base — Robot Model & Utilities

**Files:** `include/base/humanoid.h`, `include/base/interfaces.h`, `src/base/humanoid.cpp`

#### `Humanoid`

The central entry point. Constructing a `Humanoid` performs the full initialisation chain:

1. **Loads the URDF** (29-DOF G1 model) via Pinocchio, including collision geometries from STL meshes.
2. **Parses `g1.yaml`** to determine the active profile, which locked joints to freeze, joint-limit overrides, and end-effector frame definitions.
3. **Adds end-effector frames** (`L_ee`, `R_ee`) by attaching a frame at `parent_joint + offset` for each arm.
4. **Builds a reduced model** by freezing joints listed in `lock_categories` + `lock_joints`. For the default `arms_with_waist` profile this leaves an 11-DOF model (2×4 upper arm + 2×3 wrist + 3 waist).
5. **Applies joint-limit overrides** to the reduced model.
6. **Instantiates the tool chain:**
   - `HumanoidIK* ik` — inverse kinematics
   - `IKTrajTracker* controller` — trajectory-tracking via per-step IK
   - `JointSpacePlanner* motion_planner` — OMPL RRT* joint-space planner

`Humanoid::grav_ff(JointState)` returns gravity-compensation torques computed via Pinocchio's `computeGeneralizedGravity()`. `Humanoid::get_joint_names()` returns the names of the active (reduced) model.

#### Joint Categorisation (full 29-DOF URDF)

| Group | Joints | Count |
|-------|--------|-------|
| Left leg | hip (pitch/roll/yaw), knee, ankle (pitch/roll) | 6 |
| Right leg | hip (pitch/roll/yaw), knee, ankle (pitch/roll) | 6 |
| Waist | yaw, roll, pitch | 3 |
| Left upper arm | shoulder (pitch/roll/yaw), elbow | 4 |
| Right upper arm | shoulder (pitch/roll/yaw), elbow | 4 |
| Left wrist | pitch, roll, yaw | 3 |
| Right wrist | pitch, roll, yaw | 3 |
| Left gripper | thumb (×3), middle (×2), index (×2) | 7 |
| Right gripper | thumb (×3), middle (×2), index (×2) | 7 |

Total: 29 actuated DOF. Categories serve as IDs — what is actually controlled is determined by the active profile.

#### Data Structures

```cpp
struct JointState {
    std::vector<std::string> name;
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> effort;
};

struct RobotConfig {
    std::string asset_file  = "../assets/g1/g1_29dof_with_hand_rev_1_0.urdf";
    std::string asset_root  = "../assets/g1/";
    std::string config_file = "../config/g1.yaml";   // path to YAML profile
    int NUM_DOF = 29;
};
```

Utility functions `jointstate_to_vectors()` and `vectors_to_jointstate()` convert between `JointState` and Pinocchio-native `Eigen::VectorXd` representations using the model's joint indexing.

---

### arm_ik — Inverse Kinematics

**Files:** `include/arm_ik/robot_arm_ik.h`, `src/arm_ik/robot_arm_ik.cpp`, plus `utils.h` and `weighted_moving_filter.h`

#### `HumanoidIK`

Solves inverse kinematics as a **nonlinear optimisation** using CasADi + IPOPT.

##### How It Works

1. **Symbolic model**: the Pinocchio model is cast to `casadi::SX` symbolic type. Forward kinematics is evaluated symbolically to express end-effector poses as functions of joint angles.

2. **Cost function**: a weighted sum of four terms:

   | Term | Weight | Purpose |
   |------|--------|---------|
   | Translation error (L2) | 50.0 | reach the target position |
   | Rotation error (L2) | 3.0 | match target orientation |
   | Joint regularisation | 0.02 | prefer solutions near zero configuration |
   | Smoothness `(q − q_last)` | 0.1 | penalise large jumps between consecutive solves |

   The SE(3) error uses `log6(T_current.actInv(T_target))`, the Lie-algebraic error.

3. **Three separate optimisers**: `opti_` (both arms), `opti_l_` (left only), `opti_r_` (right only). Each has its own CasADi `Opti` problem, variables, and parameters.

4. **Joint limits**: enforced as box constraints from the reduced model + any `joint_limit_overrides` from `g1.yaml`.

5. **IPOPT settings**: max 50 iterations, tolerance 1e-6, warm-start enabled for real-time performance.

##### Self-Collision Detection

On initialisation, all collision pairs from the geometry model are registered. Adjacent links (parent-child in the kinematic tree) are filtered out to avoid spurious collisions between connected bodies. `check_collision(q)` returns whether configuration `q` is in collision; the optional `collision*` out-parameter on `solve_ik` reports it for the returned solution.

##### Solution Smoothing

A `WeightedMovingFilter` with weights `[0.4, 0.3, 0.2, 0.1]` (window size 4) smooths the IK output, reducing jitter at the cost of slight lag. `HumanoidIK::reset()` clears the filter buffer (called e.g. when an action server completes a goal).

##### External Wrench Support

After solving IK, the solver computes feedforward joint torques compensating for external wrenches at the end-effectors using the frame Jacobian transpose: `tau = J^T · F_ext`. Wrench inputs are the optional `EE_efrc_L` / `EE_efrc_R` arguments.

#### Public API

```cpp
// Both arms simultaneously
JointState solve_ik(
    const Eigen::Matrix4d& left_wrist,
    const Eigen::Matrix4d& right_wrist,
    const Eigen::VectorXd* current_q   = nullptr,
    const Eigen::VectorXd* current_dq  = nullptr,
    const Eigen::VectorXd* EE_efrc_L   = nullptr,
    const Eigen::VectorXd* EE_efrc_R   = nullptr,
    double* l_pos_err = nullptr, double* l_rot_err = nullptr,
    double* r_pos_err = nullptr, double* r_rot_err = nullptr,
    bool*   collision = nullptr
);

// Single arm (left=true → left arm)
JointState solve_ik(
    const Eigen::Matrix4d& wrist,
    const bool left = false,
    const Eigen::VectorXd* current_q  = nullptr,
    const Eigen::VectorXd* current_dq = nullptr,
    const Eigen::VectorXd* EE_efrc    = nullptr,
    double* pos_err   = nullptr,
    double* rot_err   = nullptr,
    bool*   collision = nullptr
);

bool check_collision(const Eigen::VectorXd& q);
void reset();   // clear smoothing filter
```

---

### arm_control — Controllers

**Files:** `include/arm_control/`, `src/arm_control/`

#### `ArmController` (base class)

Provides:

- `update(JointState)` — refreshes joint positions/velocities, runs forward kinematics, caches end-effector poses (`current_left_ee_pose_`, `current_right_ee_pose_`), end-effector velocities in world (`Motion`), and frame Jacobians (`J_left_ee_`, `J_right_ee_`).
- Gravity compensation via `pinocchio::computeGeneralizedGravity()` (cached in `grav_torques`).
- `control_no_arms(JointState)` — publishes only gravity feedforward (used when no goal is active).
- Accessors for the cached EE poses, EE velocities, and per-arm error magnitudes.

#### `ImpedanceController`

Task-space impedance control with body-frame error and **null-space stiffness** + **per-tick torque-rate limiting**.

**Control law (per arm):**

```
tau_arm = J_body^T · (Kp ⊙ e_twist + Kd ⊙ e_vel)
        + nullspace_torques
        + gravity_compensation
```

- `e_twist = log6(T_current^{-1} · T_desired)` — SE(3) pose error in body frame
- `e_vel   = v_desired − v_current` — velocity error in body frame
- For dual-arm mode the two arm Jacobians are stacked (`J_dual_` is 12 × nv) and a null-space projector preserves a desired joint configuration (`q_nullspace_desired_`) with stiffness/damping `Kp_nullspace = 25`, `Kd_nullspace = 5`.
- The output torque change per tick is clamped to `max_torque_rate / update_frequency = 50 / 100 = 0.5 N·m`.

Default gains: `Kp_linear = 50`, `Kp_angular = 5`, `Kd_linear = 5`, `Kd_angular = 2`.

API: `control_both_arms()`, `control_left_arm()`, `control_right_arm()`, `set_nullspace_target(q_d)`, `reset()`.

#### `IKTrajTracker`

An alternative controller that delegates to the IK solver each tick rather than computing torques directly. Given a desired end-effector pose, it:

1. Calls `HumanoidIK::solve_ik()` to get joint-level position + effort commands.
2. Computes an error metric combining linear and angular terms.

This is the controller used by `Humanoid` and the visual-servo / action-server nodes, where trajectory waypoints are tracked sequentially. Default gains: `Kp_linear = 30`, `Kp_angular = 5`, `Kd/Ki = 0`, `Ki_max = 5.0` (anti-windup).

---

### arm_mp — Motion Planning

**Files:** `include/arm_mp/`, `src/arm_mp/`

The library ships three planner classes, all returning `(joint_trajectory, pose_trajectory)` pairs (the polynomial generator returns just the pose trajectory).

#### `PolynomialTrajectoryGenerator`

Generates smooth trajectories on SE(3) using **polynomial interpolation in the Lie algebra**.

**Method:**

1. Compute the relative transform: `T_rel = T_start^{-1} · T_goal`
2. Take the matrix logarithm: `xi = log(T_rel)` — a 4×4 element of se(3)
3. Extract the 6D twist via the `vee` map: `[v; omega]`
4. Fit a polynomial (default order 5) to interpolate from `[0,…,0]` to the twist, with boundary conditions on velocity and acceleration
5. Evaluate the polynomial at uniform time steps
6. Map each twist back to SE(3) via `T(s) = T_start · exp(hat(xi(s)))`

**Time scaling:** duration = translational distance / `MAX_LIN_VEL` (default 0.1 m/s). Velocities and accelerations are scaled to the `[0,1]` parameter domain.

**Constraint system:** the 5th-order polynomial satisfies 6 boundary conditions — position, velocity, and acceleration at both start and goal. The constraint matrix is pre-computed and pseudo-inverted at construction time for efficiency.

#### `JointSpacePlanner` *(default planner used by `Humanoid`)*

OMPL **RRT***-based joint-space motion planner. Matches the polynomial generator's interface but plans in the reduced model's `nq`-dimensional joint space.

**Construction parameters:**

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `left_arm` | `false` | Which arm's EE pose maps to/from joint configurations. |
| `step_size` | 0.05 | RRT* extension step (radians, joint space). |
| `goal_bias` | 0.1 | Probability of sampling the goal directly. |
| `rewire_factor` | 1.1 | RRT* rewire radius factor. |
| `solve_time_seconds` | 1.0 | Time budget per planning call. |
| `goal_tolerance` | 0.01 | Goal-region radius (joint space). |
| `validity_resolution` | 0.005 | Edge-validity check resolution. |

**Pipeline:**

1. `poseToJointConfig()` — call `HumanoidIK::solve_ik()` to map the SE(3) goal to a joint configuration; reject if IK error exceeds threshold.
2. `runRRTStar()` — set up an OMPL `RealVectorStateSpace` with bounds from the Pinocchio model, validity check via `HumanoidIK::check_collision()`, and run RRT* for the time budget.
3. `resamplePath()` — densely re-sample the OMPL solution to a fixed step.
4. `configsToPoses()` — forward-kinematics each waypoint to recover the EE pose trajectory.

Velocity/acceleration arguments are accepted for API parity but ignored (RRT* is kinematic). `setLeftArm(bool)` switches arms at runtime.

#### `VisualServoPlanner` *(extends `JointSpacePlanner`)*

Adds a **two-phase approach** for manipulation tasks:

```
Phase 1 (RRT*):     current pose  →  intermediate pose
                    (offset along −x axis of target by approach_offset = 5 cm)

Phase 2 (linear):   intermediate  →  target
                    (linear interpolation in SE(3), final_leg_steps = 20)
```

This guarantees the end-effector approaches the target **head-on along the contact normal (x-axis)**, which is critical for pressing a button or grasping. When the start is already within `approach_offset` of the target, Phase 1 is skipped.

The final leg uses linear interpolation rather than the polynomial generator since it is short and the approach direction is the load-bearing constraint.

---

## g1_pilot ROS 2 Nodes

All nodes accept the standard parameters (`asset_file`, `asset_root`, `config_file`, `num_dof`) and load the URDF + `g1.yaml` from `g1_description` / `g1_pilot` package shares.

### `controller` — `ControlTrajectory` action server

Action server that tracks an SE(3) `nav_msgs/Path` waypoint-by-waypoint using `IKTrajTracker`. Pre-empts any active goal when a new one arrives.

| | Topic / Action | Type |
|---|---|---|
| Action srv | `trajectory_controller` | `humanoid_manipulation_interfaces/ControlTrajectory` |
| Sub | `feedback` | `sensor_msgs/JointState` |
| Pub | `position_control` | `sensor_msgs/JointState` |
| Pub | `left_ee_pose`, `right_ee_pose` | `geometry_msgs/PoseStamped` (in `pelvis` frame) |

Control loop: 50 ms. Waypoints pop when error < `waypoint_error_margin` (default 0.007). Goal is rejected if the final pose fails workspace bounds (`isWithinLimits`). On success/cancel the IK solver's smoothing filter is reset.

### `joint_controller` — `ControlJointTrajectory` action server

Same pattern as `controller`, but the goal is a `trajectory_msgs/JointTrajectory` (joint-space) and the controller commands raw positions + gravity feedforward effort instead of running IK each tick.

| | Topic / Action | Type |
|---|---|---|
| Action srv | `trajectory_controller` | `humanoid_manipulation_interfaces/ControlJointTrajectory` |
| Sub | `feedback` | `sensor_msgs/JointState` |
| Pub | `position_control` | `sensor_msgs/JointState` |
| Pub | `left_ee_pose`, `right_ee_pose` | `geometry_msgs/PoseStamped` |

Default `waypoint_error_margin` is 0.003 (radians, RMS over commanded joints).

### `plan_trajecory` — `PlanJointTrajectory` service server

Computes a joint-space trajectory from a target pose using `Humanoid::motion_planner` (OMPL RRT*). Uses the **live** EE pose (from `left_ee_pose` / `right_ee_pose` topics) as the start pose, so it can be called repeatedly during execution to replan from the current state.

| | Topic / Service | Type |
|---|---|---|
| Service | `plan_trajectory` | `humanoid_manipulation_interfaces/PlanJointTrajectory` |
| Sub | `/left_ee_pose`, `/right_ee_pose` | `geometry_msgs/PoseStamped` |
| Pub | `traj` | `nav_msgs/Path` (visualisation) |

Uses TF2 to transform the request's `target_pose` into the `pelvis` frame.

### `visual_servo` — Full visual-servoing pipeline (dual-arm, in-process)

Combines motion planning, trajectory tracking, and real-time control inside a single node. Subscribes to **separate** left/right goal topics (suffixed `/left`, `/right`), each gated by an independent cooldown.

| | Topic | Type |
|---|---|---|
| Sub | `<goal_pose_topic>/left` | `geometry_msgs/PoseStamped` |
| Sub | `<goal_pose_topic>/right` | `geometry_msgs/PoseStamped` |
| Sub | `feedback` | `sensor_msgs/JointState` |
| Pub | `position_control` | `sensor_msgs/JointState` |
| Pub | `traj` | `nav_msgs/Path` |

Goal cooldown is `goal_cooldown` (default 1 s). At 20 Hz the control loop pops the next waypoint when EE error < 2 cm. When idle the node publishes gravity compensation only.

The default `visual_servo.launch.py` remaps `<goal_pose_topic>/right` → `/track3d/selected_normal_pose_filtered_upright` to plug into the perception stack.

### `impede_arms` — Impedance control

Subscribes to feedback joint states and computes impedance-control torques. Accepts goal poses via `goal_pose`.

| | Topic | Type |
|---|---|---|
| Sub | `feedback` | `sensor_msgs/JointState` |
| Sub | `goal_pose` | `geometry_msgs/PoseStamped` |
| Pub | `effort_control` | `sensor_msgs/JointState` |

### `grav_ff` — Gravity feedforward

Reads current joint state, computes gravity compensation via Pinocchio, publishes it.

| | Topic | Type |
|---|---|---|
| Sub | `feedback` | `sensor_msgs/JointState` |
| Pub | `effort_control` | `sensor_msgs/JointState` |

### `joint_traj_publisher` — Trajectory demo

Re-plans a sample trajectory every 3 s and publishes IK solutions for each waypoint along with the path. Useful for sanity-checking the planner + IK loop without a perception pipeline.

### `ik_joint_state_publisher` — IK demo

Publishes IK solutions at 10 Hz for a fixed sinusoidal target. Useful for verifying the IK + URDF loop in isolation.

### Launch Files (`g1_pilot/launch/`)

| Launch file | Brings up |
|-------------|-----------|
| `controller.launch.py` | `joint_controller` (ControlJointTrajectory action server). Optional `display.launch.py` via `PublishTF`. |
| `plan_trajectory.launch.py` | `plan_trajecory` (PlanJointTrajectory service). |
| `visual_servo.launch.py` | `visual_servo`, with right goal topic remapped to `/track3d/selected_normal_pose_filtered_upright`. |
| `visual_servo_aruco.launch.py` | `visual_servo` + RealSense + ArUco detector (requires `realsense2_camera`, `hand_eye_calibration`). |
| `ik_demo.launch.py` | `ik_joint_state_publisher`. |
| `grav_ff.launch.py` | `grav_ff`. |
| `impede_arms_demo.launch.py` | `impede_arms`. |
| `joint_traj_demo.launch.py` | `joint_traj_publisher`. |

All launch files expose namespace, topic remap, and `PublishTF` (toggle to include `g1_description/display.launch.py`) arguments.

---

## humanoid_arm_skills (Python)

`humanoid_arm_skills` is a `ament_python` package providing high-level skills that compose `g1_pilot`'s service + action interfaces.

### Console Scripts (entry points in `setup.py`)

| Script | Module | Purpose |
|--------|--------|---------|
| `humanoid_arm_vs` | `humanoid_arm_visual_servoing` | Visual-servoing **action server** (the main skill). |
| `pose_filtering` | `pose_filtering` | Generic moving-average + outlier-rejection pose filter (`PoseStamped` and `Pose2D`). |
| `noisy_pose_publisher` | `noisy_pose_publisher` | Test publisher emitting Gaussian-noisy `PoseStamped` messages. |

### `humanoid_arm_vs` — `VisualServoingArm` action server

Drives the arm to a perceived target (a tracked surface normal) by **dynamically re-planning** as the target moves or as tracking error sustains.

**Pipeline per goal:**

1. Reset the visual tracker (`/track_3d/reset`); forward `target_description` to the VLM via `keyboard_input_topic` so it can ground the target.
2. Wait for the first filtered target pose + current EE pose, then call the `PlanJointTrajectory` service.
3. Send the returned trajectory to the `ControlJointTrajectory` action.
4. In the main loop (default 2 Hz), monitor:
   - Goal-pose drift (position Δ ≥ `replan_goal_position_delta`, orientation Δ ≥ `replan_goal_orientation_delta_deg`) → trigger replan.
   - Sustained controller error > `replan_error_threshold` for > `replan_error_duration` s → trigger replan.
   - Tracking lost → report in feedback.
   - Goal succeeded / cancelled / timed out → terminate.
5. Replans are gated by a `ReplanPolicyChain` (no active goal, EE/target unavailable, EE within `replan_skip_distance`, post-goal cooldown, replan cooldown, replan in progress).

**Action interface** (`humanoid_manipulation_interfaces/VisualServoingArm`):

- Goal: `string target_description`, `bool left_arm`
- Feedback: `distance_to_goal`, `orientation_error_deg`, `tracking_lost`, `elapsed_time`, `current_goal`
- Result: `success`, `message`, `final_distance`, `orientation_error_deg`

**Key parameters** (all dynamic via `dynamic_params`):

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `goal_rate` | 2.0 Hz | Main loop rate. |
| `action_timeout` | 600 s | Per-goal timeout. |
| `replan_error_threshold` | 0.01 | Joint-space RMS error to start replan timer. |
| `replan_error_duration` | 5.0 s | Sustained-error window before replan fires. |
| `replan_cooldown` | 5.0 s | Minimum gap between replans. |
| `replan_skip_distance` | 0.12 m | Don't replan when EE is closer than this. |
| `replan_goal_position_delta` | 0.01 m | Position drift to trigger replan. |
| `replan_goal_orientation_delta_deg` | 15° | Orientation drift to trigger replan. |
| `post_goal_cooldown` | 1.0 s | Cooldown after a successful goal. |
| `arm_safety_distance_{x,y,z}` | (-0.01, 0.0, 0.0) m | Local-frame offset applied to the target before planning. |

Threading: a `ReentrantCallbackGroup` allows overlapping subscriber/timer callbacks; three locks (`_replan_lock`, `_control_lock`, `_trajectory_lock`) guard internal state.

### `pose_filtering`

Generic per-axis moving-average filter with outlier rejection.

- Supports `PoseStamped` and `Pose2D` (selected via `msg_type` parameter).
- Position outliers: `> position_outlier_std × σ` from mean of buffer.
- Orientation outliers (`PoseStamped`): angular distance > `orientation_outlier_rad` from buffer mean (eigenvector quaternion average, Markley method).
- Orientation outliers (`Pose2D`): circular-mean angle distance.
- Optionally TF-transforms input into `target_frame_arm` before filtering.
- Publishes a second `<output>_upright` topic for `PoseStamped` (yaw-only, Z-up version) — this is what the visual-servo skill consumes.

Buffer is reset if no input arrives within `lost_timeout` (default 5 s).

### `noisy_pose_publisher`

Test fixture. Publishes a configurable mean pose with Gaussian noise added to position and orientation. Useful to sanity-check `pose_filtering` without real perception.

### Launch File

`humanoid_arm_skills/launch/humanoid_arm_visual_servoing.launch.py` brings up the full skill stack:

1. `g1_description/display.launch.py` (conditional on `PublishTF`)
2. `g1_pilot/controller.launch.py` (joint trajectory action server)
3. `g1_pilot/plan_trajectory.launch.py` (planner service)
4. `pose_filtering` node (filters `track_3d/selected_normal_pose` → `…_filtered/arm` and `_filtered/arm_upright`)
5. `humanoid_arm_vs` action server

---

## humanoid_manipulation_interfaces

ROS 2 interface package generated via `rosidl_default_generators`. Contains 3 actions + 2 services that form the contract between `humanoid_arm_skills` and `g1_pilot`.

### Actions (`action/`)

| Action | Goal | Feedback | Result |
|--------|------|----------|--------|
| `ControlTrajectory` | `bool left_arm`, `nav_msgs/Path trajectory` | `float64 current_error`, `uint32 waypoints_remaining`, `uint32 total_waypoints` | `bool success`, `float64 final_error` |
| `ControlJointTrajectory` | `bool left_arm`, `trajectory_msgs/JointTrajectory trajectory` | `float64 current_error`, `uint32 waypoints_remaining`, `uint32 total_waypoints` | `bool success`, `float64 final_error` |
| `VisualServoingArm` | `string target_description`, `bool left_arm` | `float64 distance_to_goal`, `float64 orientation_error_deg`, `bool tracking_lost`, `float64 elapsed_time`, `geometry_msgs/PoseStamped current_goal` | `float64 final_distance`, `float64 orientation_error_deg`, `bool success`, `string message` |

### Services (`srv/`)

| Service | Request | Response |
|---------|---------|----------|
| `PlanTrajectory` | `geometry_msgs/PoseStamped target_pose`, `geometry_msgs/PoseStamped start_pose`, `bool left_arm` | `bool success`, `nav_msgs/Path trajectory` |
| `PlanJointTrajectory` | `geometry_msgs/PoseStamped target_pose`, `geometry_msgs/PoseStamped start_pose`, `bool left_arm` | `bool success`, `trajectory_msgs/JointTrajectory trajectory` |

The `_Joint*` variants exchange joint-space trajectories; the SE(3) variants exchange `nav_msgs/Path`.

---

## g1_description

Standalone description package — lets other packages declare a clean dependency on the URDF without pulling in the C++ build.

| Item | Notes |
|------|-------|
| URDFs | `assets/g1/`: `g1_29dof_with_hand_rev_1_0.urdf` (raw 29-DOF model) plus `_ros.urdf` and `_ros_ctrl_viz.urdf` (xacro outputs used for dual-namespace RViz visualisation). |
| Meshes | 160 STL files under `assets/g1/meshes/`. |
| RViz config | `rviz/manipulation.rviz`. |
| `display.launch.py` | Two `robot_state_publisher` instances: the `feedback` one publishes the real TF tree from `/feedback`; the `control_viz` one publishes the *commanded* configuration under a `control/` frame prefix from `/position_control`. A static `base_link ↔ control/base_link` transform overlays them. |

The visual servo + controller stack uses this dual visualisation so RViz shows both the actual and commanded arm configurations side-by-side.

---

## Design Decisions

### Configuration-driven joint reduction

The library used to hard-code which joints to freeze. It now reads `g1.yaml`, where named profiles select `lock_categories` + `joint_limit_overrides` + `end_effectors`. Switching from "arms only" to "arms + waist" is a YAML edit, not a recompile, and the same library can support related humanoids by swapping the URDF + YAML.

### OMPL RRT* as the default planner

`Humanoid` instantiates `JointSpacePlanner` (OMPL RRT*) as the default `motion_planner`. Compared to the polynomial generator it:

- Produces collision-free joint trajectories rather than collision-blind SE(3) interpolations.
- Naturally handles joint limits and self-collisions through OMPL's validity checker (`HumanoidIK::check_collision`).
- Costs more per call (≤ 1 s solve budget by default) but is amortised by the skill layer's 5 s replan cooldown.

`VisualServoPlanner` keeps RRT* for the bulk approach and switches to a short linear final leg, so the contact-axis approach direction is preserved.

### Lie-group SE(3) trajectories

Trajectories — both polynomial and the visual-servo final leg — are generated on SE(3) via the exponential map rather than interpolating position and orientation separately. This avoids gimbal lock, ensures valid rotation matrices at every point, and produces geodesic motions in the group.

### Skill ↔ controller decoupling via actions/services

`humanoid_arm_skills` does not depend on the `g1_pilot` C++ library — it talks to `g1_pilot` exclusively through the `humanoid_manipulation_interfaces` actions/services. This lets the skill layer:

- Replan from any client without restarting the controller.
- Pre-empt and re-issue trajectories cleanly (the action server handles preemption explicitly).
- Be tested in Python against fake controllers that implement the same interfaces.

### Dual build system

CMake supports two modes:

- **With ROS 2** — detected via `AMENT_PREFIX_PATH` / `COLCON_PREFIX_PATH` / `ROS_DISTRO`. Builds the library + all 8 ROS executables.
- **Standalone C++** — builds only the library, installs to a local `install/` directory. No ROS dependencies needed.

### Dual-arm impedance with null-space stiffness

The impedance controller stacks both arm Jacobians into a 12 × nv operational-space Jacobian and projects a null-space joint-impedance term into the kernel. Combined with a per-tick torque-rate limit (default 0.5 N·m/tick at 100 Hz), this keeps the robot's posture controlled without fighting the task command.

---

## Usage Guide

### As a C++ library

After building (see `README.md`):

```cpp
#include <g1_pilot/g1_pilot.h>
using namespace HumanoidPilot;
```

#### Initialise the robot

```cpp
RobotConfig config;
config.asset_file  = "/path/to/g1_29dof_with_hand_rev_1_0.urdf";
config.asset_root  = "/path/to/g1/meshes/";
config.config_file = "/path/to/config/g1.yaml";
config.NUM_DOF     = 29;

Humanoid arm(&config);
```

#### Solve IK for both arms

```cpp
Eigen::Matrix4d left_target  = Eigen::Matrix4d::Identity();
left_target.block<3,1>(0,3)  = Eigen::Vector3d(0.2,  0.2, 0.1);

Eigen::Matrix4d right_target = Eigen::Matrix4d::Identity();
right_target.block<3,1>(0,3) = Eigen::Vector3d(0.2, -0.2, 0.1);

JointState result = arm.ik->solve_ik(left_target, right_target);
// result.name      — joint names (active reduced model)
// result.position  — joint angles (rad)
// result.effort    — gravity feedforward + (optional) external-wrench torques (N·m)
```

#### Solve IK for a single arm

```cpp
JointState res_r = arm.ik->solve_ik(right_target, /*left=*/false);
JointState res_l = arm.ik->solve_ik(left_target,  /*left=*/true);
```

#### Warm-start with current state

```cpp
Eigen::VectorXd current_q = /* from encoders */;
JointState result = arm.ik->solve_ik(left_target, right_target, &current_q);
```

#### Plan a joint-space trajectory (RRT*)

```cpp
Eigen::Matrix4d start = Eigen::Matrix4d::Identity();
start.block<3,1>(0,3) = Eigen::Vector3d(0.20, -0.15, 0.10);

Eigen::Matrix4d goal  = Eigen::Matrix4d::Identity();
goal.block<3,1>(0,3)  = Eigen::Vector3d(0.35, -0.15, 0.25);

arm.motion_planner->setLeftArm(false);
auto [joint_traj, pose_traj] = arm.motion_planner->planTrajectory(&goal, &start);

for (const auto& q : joint_traj) {
    /* send q to actuators … */
}
```

#### Gravity feedforward only

```cpp
JointState s;
s.name = arm.get_joint_names();
s.position.assign(s.name.size(), 0.0);
s.velocity.assign(s.name.size(), 0.0);
s.effort.assign(s.name.size(),   0.0);

JointState g = arm.grav_ff(s);
// g.effort holds gravity-compensation torques for the active reduced model.
```

#### Helpers (ROS-side, in `helper_funcs.h`)

```cpp
#include "helper_funcs.h"

Eigen::Matrix4d T = create_se3(1.0, 0.0, 0.0, 0.0,   // qw, qx, qy, qz
                                0.2, -0.15, 0.1);    // tx, ty, tz

Eigen::Matrix4d T2 = create_se3(Eigen::Quaterniond(1,0,0,0),
                                 Eigen::Vector3d(0.2,-0.15,0.1));

// Workspace check (defaults: x∈[0.1,0.6], y∈[-0.6,0.6], z∈[0.0,1.0], |angle|≤π/3)
std::string reason;
bool ok = isWithinLimits(T, reason);
```

### With ROS 2

#### Visualisation only

```bash
ros2 launch g1_description display.launch.py
```

Opens RViz with the dual-namespace G1 model (`feedback` + `control` overlays).

#### IK demo

```bash
ros2 launch g1_pilot ik_demo.launch.py
```

Then publish a goal pose:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'pelvis'}, pose: {position: {x: 0.3, y: -0.15, z: 0.2}, orientation: {w: 1.0}}}"
```

#### Controller + planner standalone

```bash
ros2 launch g1_pilot controller.launch.py        # joint_controller action server
ros2 launch g1_pilot plan_trajectory.launch.py   # PlanJointTrajectory service
```

Then call the planner and send the trajectory through the controller — see `humanoid_arm_skills/humanoid_arm_visual_servoing.py` for an end-to-end example.

#### Visual servoing (in-process node)

```bash
ros2 launch g1_pilot visual_servo.launch.py
```

Send goals on `/arm/goal_pose/left` and `/arm/goal_pose/right` (or pipe perception into the remapped `/track3d/selected_normal_pose_filtered_upright`):

```bash
ros2 topic pub --once /arm/goal_pose/right geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'pelvis'}, pose: {position: {x: 0.4, y: -0.15, z: 0.2}, orientation: {w: 1.0}}}"
```

The planned trajectory appears in RViz on the `/traj` topic.

#### Visual servoing skill (action server with replanning)

```bash
ros2 launch humanoid_arm_skills humanoid_arm_visual_servoing.launch.py
```

This composes the controller, planner, and pose filter, and starts the `humanoid_arm_vs` action server. Drive it with an action client targeting `humanoid_arm_visual_servoing` of type `humanoid_manipulation_interfaces/VisualServoingArm`.

#### Visual servoing with ArUco

```bash
ros2 launch g1_pilot visual_servo_aruco.launch.py
```

Brings up the in-process visual-servo node together with a RealSense camera and ArUco detector (requires `realsense2_camera` and `hand_eye_calibration`).

#### Impedance control / gravity feedforward

```bash
ros2 launch g1_pilot impede_arms_demo.launch.py
ros2 launch g1_pilot grav_ff.launch.py
```

Both consume `feedback` and publish `effort_control`.

#### Trajectory demo

```bash
ros2 launch g1_pilot joint_traj_demo.launch.py
```

Re-plans every 3 s and publishes IK solutions per waypoint.

---

## ROS 2 Topic / Service / Action Interface

### Topics

| Direction | Topic | Type | Used by |
|-----------|-------|------|---------|
| In | `feedback` | `sensor_msgs/JointState` | all controllers |
| In | `goal_pose` (or `goal_pose/{left,right}`) | `geometry_msgs/PoseStamped` | `visual_servo`, `impede_arms` |
| Out | `position_control` | `sensor_msgs/JointState` | `controller`, `joint_controller`, `visual_servo`, `ik_joint_state_publisher`, `joint_traj_publisher` |
| Out | `effort_control` | `sensor_msgs/JointState` | `impede_arms`, `grav_ff` |
| Out | `traj` | `nav_msgs/Path` | planners (visualisation) |
| Out | `left_ee_pose`, `right_ee_pose` | `geometry_msgs/PoseStamped` | `controller`, `joint_controller` |
| In/Out (skill) | `track_3d/*` | various | `humanoid_arm_vs`, `pose_filtering` |
| Out (skill) | `keyboard_input` | `std_msgs/String` | `humanoid_arm_vs` (target → VLM) |

### Services

| Service | Type | Server |
|---------|------|--------|
| `plan_trajectory` | `humanoid_manipulation_interfaces/PlanJointTrajectory` | `plan_trajecory` |

### Actions

| Action | Type | Server |
|--------|------|--------|
| `trajectory_controller` | `humanoid_manipulation_interfaces/ControlTrajectory` | `controller` |
| `trajectory_controller` | `humanoid_manipulation_interfaces/ControlJointTrajectory` | `joint_controller` |
| `humanoid_arm_visual_servoing` | `humanoid_manipulation_interfaces/VisualServoingArm` | `humanoid_arm_vs` |

(Only one of the two `trajectory_controller` action servers should be running at a time; the visual-servo skill expects the joint-space variant.)

---

## Configuration & Parameters

### Common ROS Parameters (every C++ node)

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `asset_file` | `<g1_description>/assets/g1/g1_29dof_with_hand_rev_1_0.urdf` | URDF path |
| `asset_root` | `<g1_description>/assets/g1/` | mesh root |
| `config_file` | `<g1_pilot>/config/g1.yaml` | YAML profile |
| `num_dof` | 29 | full URDF DOF count |

### Workspace bounds (`isWithinLimits` in `helper_funcs.h`)

Goal poses are validated against:

| Axis | Min | Max |
|------|-----|-----|
| X | 0.1 m | 0.6 m |
| Y | −0.6 m | 0.6 m |
| Z | 0.0 m | 1.0 m |
| Rotation magnitude | — | 1.04 rad (≈ π/3) |

(All in `pelvis` frame.)

### Controller gains (defaults, set programmatically)

**`IKTrajTracker`** (visual-servo / `controller`): `Kp_linear = 30`, `Kp_angular = 5`, `Kd/Ki = 0`, `Ki_max = 5.0`.

**`ImpedanceController`** (`impede_arms`): `Kp_linear = 50`, `Kp_angular = 5`, `Kd_linear = 5`, `Kd_angular = 2`. Null-space: `Kp_nullspace = 25`, `Kd_nullspace = 5`. Torque rate limit: `max_torque_rate = 50 N·m/s` at `update_frequency = 100 Hz`.

### Planner defaults

| Parameter | Default | Class |
|-----------|---------|-------|
| Polynomial order | 5 | `PolynomialTrajectoryGenerator` |
| `MAX_LIN_VEL` | 0.1 m/s | poly + visual-servo |
| `MAX_ANG_VEL` | 4.0 rad/s | poly + visual-servo |
| `MAX_LIN_ACC` | 0.05 m/s² | poly + visual-servo |
| `MAX_ANG_ACC` | 4.0 rad/s² | poly + visual-servo |
| Time step | 0.05 s | poly |
| `step_size` | 0.05 rad | OMPL RRT* |
| `goal_bias` | 0.1 | OMPL RRT* |
| `solve_time_seconds` | 1.0 s | OMPL RRT* |
| `goal_tolerance` | 0.01 rad | OMPL RRT* |
| `validity_resolution` | 0.005 rad | OMPL RRT* |
| `approach_offset` | 0.05 m | `VisualServoPlanner` |
| `final_leg_steps` | 20 | `VisualServoPlanner` |

### Skill parameters

See [`humanoid_arm_vs`](#humanoid_arm_vs--visualservoingarm-action-server) above for the full table.

---

## Testing

The library ships Google Test suites:

```bash
# Build with testing enabled (default)
colcon build

# Run all tests
cd build/g1_pilot
ctest

# Or run individually
./test_ik           # IK solver tests
./test_control      # G1 + controller initialisation tests
./test_mp_ik        # OMPL trajectory generation + IK integration
```

### Test Coverage

| Test | Verifies |
|------|----------|
| `test_ik` | Dual-arm IK solve, timing, CasADi integration. |
| `test_control` | `Humanoid` initialisation, controller setup. |
| `test_mp_ik` | `JointSpacePlanner` trajectory generation, IK along the trajectory. |
| `math_unittest` *(disabled)* | Polynomial math: power vectors, differentiation matrices, evaluation. |
| `line_unittest` *(disabled)* | Boundary conditions for the polynomial trajectory generator. |

A Python visualisation script (`test/arm_mp/line_viz.py`) plots 3D trajectories exported as CSV from the tests.
