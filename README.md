# LPG Cylinder Loading/Unloading Simulation with ROS2 & MoveIt2

## Project Overview
This project simulates an automated robotic system that loads and unloads LPG cylinders from a truck.  
- **Loading Operation:** Picks cylinders from a conveyor and loads them onto a truck in a predefined stacking order.  
- **Unloading Operation:** Removes cylinders from the truck and places them back on the conveyor.  

The system integrates **ROS2, MoveIt2, Gazebo simulation, and optional perception modules** for dynamic cylinder detection.

---

## Key Features
- **Manipulator Motion Planning:** Collision-free path planning with MoveIt2.  
- **Dynamic Environment Handling:** Gazebo world with truck, conveyor, and cylinders.  
- **Perception Integration:** Detects cylinder positions and orientations using cameras or LiDAR.  
- **Task Management:** Orchestrates pick-and-place sequences.  
- **Gripper Control:** ROS2 controllers handle grasping and releasing cylinders.  
- **Visualization:** RViz2 visualizes robot, trajectories, and planning scene.

---

## Workspace & Package Structure

ros2_ws/
├── src/
│ ├── clu_description/ # URDF/Xacro, meshes, robot configuration
│ ├── clu_moveit/ # MoveIt2 planning setup
│ ├── clu_environment/ # Gazebo worlds, truck & cylinder models
│ ├── clu_perception/ # Cylinder detection (CV/PCL)
│ ├── clu_manager/ # Pick/place sequence orchestration
│ └── clu_control/ # Gripper and robot controllers

---

## System Workflow

1. **Perception Module**
   - Captures sensor data from the conveyor/truck area.
   - Detects cylinder positions and orientations.
   - Publishes `geometry_msgs/PoseArray` on `/cylinder_poses`.

2. **Task Manager**
   - Subscribes to `/cylinder_poses`.
   - Determines the next cylinder to pick based on order.
   - Sends pick/place requests to MoveIt2.

3. **MoveIt2 Planner**
   - Receives target poses for pick and place.
   - Updates the planning scene with collision objects.
   - Computes collision-free trajectories using RRTConnect, PRM, CHOMP, or STOMP.
   - Applies orientation constraints to keep cylinders upright.

4. **Gripper & Control**
   - Executes trajectories via `ros2_control`.
   - Opens/closes gripper for secure pick and release.

5. **Execution Loop**
   - For **loading**: Conveyor → Robot → Truck.
   - For **unloading**: Truck → Robot → Conveyor.
   - Returns robot to home pose after each cycle.

**ROS2 Node & Topic Flow**

[Gazebo Cylinders]
↓ (sensor)
[Perception Node] ---> /cylinder_poses ---> [Task Manager]
↓
[MoveIt Planner] <--- pick/place requests --- [Task Manager]
↓
[Robot + Gripper Controller] <--- joint trajectories & gripper commands --- [MoveIt Planner]


## Installation & Setup
1. **Install ROS2 Humble/Iron and dependencies:**
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-moveit
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-cv-bridge ros-humble-pcl-ros
Clone this repository into your workspace:


## Technical Considerations
Collision Avoidance: Handled automatically by MoveIt2.

Orientation Constraint: Cylinders are kept upright during motion.

Dynamic Updates: Perception updates the planning scene with current cylinder positions.

Parallelization: Next pick can be precomputed while robot executes current trajectory.

Simulation Fidelity: Realistic cylinder mass, friction, and conveyor speed ensure accurate physics.

Safety features: collision monitoring, force feedback, emergency stop.