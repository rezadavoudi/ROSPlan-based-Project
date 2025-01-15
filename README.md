# ROSPlan-based-Project
---

## Project Description

This project implements a ROSPlan-based planning system for a mobile robot equipped with a camera and laser scanner. The system runs in a simulated Gazebo environment, where the robot autonomously navigates predefined waypoints, detects ArUco markers, and moves to the waypoint containing the marker with the smallest ID.

### Key Features

- **Autonomous Waypoint Navigation**:
  - WP0: `(x = -7.0, y = 1.5)`
  - WP1: `(x = -3.0, y = -8.0)`
  - WP2: `(x = 6.0, y = 2.0)`
  - WP3: `(x = 7.0, y = -5.0)`
  - WP4 (Initial Position): `(x = 0, y = 2.75)`

- **Marker Detection**:
  - Detects ArUco markers at waypoints and keeps track of the marker with the smallest ID.

- **ROSPlan Integration**:
  - Utilizes PDDL-based planning to sequence and execute robot actions.

---

## Usage Instructions

### 1. Clone the Repository
```bash
git clone https://github.com/rezadavoudi/ROSPlan-based-Project.git
```

### 2. Install Dependencies
Ensure all necessary ROS packages, including `aruco_ros`, `move_base`, `gazebo_ros`, and `slam_gmapping`, are installed.

### 3. Replace Marker Detection File
Replace the `marker_publish.cpp` file in the `aruco_ros` package with the modified version included in this repository.

### 4. ROSPlan and SLAM Installation
Follow the installation instructions for ROSPlan dependencies, which can be found in the official repository and AulaWeb resources.

### 5. Launch the System
- Launch the simulation environment and the robot control systems:
```bash
roslaunch assignment2_exprob main.launch
```
- Start the planning process:
```bash
roslaunch assignment2_exprob plan_sim.launch
```

### 6. Generate and Dispatch the Plan
Execute the following commands:
```bash
rosservice call /rosplan_problem_interface/problem_generation_server
rosservice call /rosplan_planner_interface/planning_server
rosservice call /rosplan_parsing_interface/parse_plan
rosservice call /rosplan_plan_dispatcher/dispatch_plan
```

### 7. View the Generated Plan
To see the planner output:
```bash
rostopic echo /rosplan_planner_interface/planner_output -p
```

---

## Actions Overview

The `Robot_Domain.pddl` file defines three primary actions:

1. **move_to**
   - Moves the robot to a target waypoint using `MoveBase`.

2. **identify**
   - Rotates the robot at the waypoint to detect ArUco markers and updates the smallest marker ID and location.

3. **verify_targets**
   - Ensures all markers are detected and navigates to the waypoint with the smallest marker ID.

---

## Package Details

### `my_rosplan_interface`
- **Action Dispatch Node** (`my_action`): Implements PDDL action dispatching.
  - **move_to**: Sends waypoint coordinates to `MoveBase` for autonomous navigation.
  - **identify**: Detects ArUco markers and updates the marker ID.
  - **verify_targets**: Directs the robot to the waypoint with the smallest marker ID.

### `aruco_ros`
- Includes the modified `marker_publish.cpp` file to handle ArUco marker detection more efficiently.

---

### Notes
- Ensure that the system is correctly initialized with SLAM and `MoveBase` before launching the planner.
- Adjust parameters in `main.launch` and `plan_sim.launch` if running the simulation on different hardware configurations.
