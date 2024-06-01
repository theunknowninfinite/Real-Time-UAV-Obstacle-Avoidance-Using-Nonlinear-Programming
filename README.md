# Real-Time UAV Obstacle Avoidance Using NonLinear Programming
This project showcases real-time obstacle avoidance in UAVs by utilizing the ModalAI Voxl 2 Sentinel Drone with the PX4 flight stack and an Intel Realsense D435i camera to precisely detect obstacles in its environment. The detected object positions are incorporated as constraints in a non-linear programming-based path planning algorithm, allowing the UAV to autonomously generate safe flight paths with a defined distance threshold around these obstacles. To ensure continuous paths, reference velocities and accelerations are calculated at each waypoint. By accounting for room dimensions and drone dynamics as constraints in the optimizer, these paths ensure efficient and secure navigation, enabling the drone to reach its designated destination accurately and reliably.

# PX4 Simulation Codes

This guide provides step-by-step instructions to launch the PX4 simulation, including setting up the necessary environment, starting essential services, and running trajectory and obstacle avoidance scripts.

## Prerequisites

Ensure the following commands are added to your `.bashrc` file to set up the ROS environment:

```bash
source /opt/ros/humble/setup.bash
source /home/root/offboard_ws/install/setup.bash
```

Reload your `.bashrc` or source it manually:

```bash
source ~/.bashrc
```

## Step-by-Step Instructions

### 1. Start uXRCE and Gazebo Bridge

Open a terminal and execute the following command to start the uXRCE and Gazebo bridge:

```bash
./start_uXRCE_and_gzbridge.sh
```

### 2. Launch PX4 Autopilot and Gazebo World

In a new terminal, run the script to launch the PX4 autopilot and the Gazebo Baylands world:

```bash
./run_baylands_total-1.sh
```

### 3. Navigate to Obstacle Avoidance Package

In another terminal, change the directory to the obstacle avoidance package:

```bash
cd final_project_ws/src/obstacle_avoidance/scripts/
```

### 4. Start Offboard Control

Before running the offboard control scripts, ensure QGroundControl is started and manually take off the drone.

#### 4.1 Run Trajectory Tracker

Video demonstrating the tracker following a square trajectory: </br>
[![YouTube](http://i.ytimg.com/vi/AAhBlBkOZPs/hqdefault.jpg)](https://www.youtube.com/watch?v=AAhBlBkOZPs)

To run the straight-line trajectory tracker between a set of goal points, execute the following command:

```bash
python3 trajectory_tracker.py
```

#### 4.2 Run Obstacle Avoidance 

To run the obstacle avoidance script that maintains a 1-meter distance threshold from obstacles, use the command:

```bash
python3 obstacle_avoidance.py
```

## Notes
- **Obstacle Position:** For the purpose of simulation, the obstacle position is defined halfway between the drones current position and the goal position. Real-time obstacle detection and avoidance is executed in the Onboard Codes scripts
