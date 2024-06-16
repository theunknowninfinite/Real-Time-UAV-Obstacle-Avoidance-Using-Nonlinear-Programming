# Real-Time UAV Obstacle Avoidance Using NonLinear Programming
This project showcases real-time obstacle avoidance in UAVs by utilizing the ModalAI Voxl 2 Sentinel Drone with the PX4 flight stack and an Intel Realsense D435i camera to precisely detect obstacles in its environment. The detected object positions are incorporated as constraints in a non-linear programming-based path planning algorithm, allowing the UAV to autonomously generate safe flight paths with a defined distance threshold around these obstacles. To ensure continuous paths, reference velocities and accelerations are calculated at each waypoint. By accounting for room dimensions and drone dynamics as constraints in the optimizer, these paths ensure efficient and secure navigation, enabling the drone to reach its designated destination accurately and reliably.

# Running on Hardware

This code can be run on a ModalAI Voxl2-based drone. For this project, a Sentinel drone was used.
- A Intel Realsense D435i camera has to be connected to the drone. A USB 2.0 connection was used for this project but it is highly recommended  to USB3.0 for the camera if possible. 

## Installing Required Softwaree 
- Install docker on the Sentinal Drone as given on ModalAI [docs](https://docs.modalai.com/voxl-docker-and-cross-installation/).

- Create a docker image with Ubuntu 20.04 and ROS Humble. The Intel RealSense SDK and respective ROS2 wrapper can be installed as explained in the [GitHub](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#installation-on-ubuntu) repository for the RealSense ROS Wrapper. This also has to be installed within the docker container.

## Setting up ROS Domain ID Overide and XRCE_DOMAIN_ID_OVERRIDE
To set up the ROS Domain ID Override and XRCE_DOMAIN_ID_OVERRIDE, follow these steps:

1. Open a terminal and navigate to your home directory:
```bash
cd ~
```

2. Open the `.bashrc` file using a text editor:
```bash
nano .bashrc
```

3. Scroll to the bottom of the file and add the following lines:
```bash
export XRCE_DOMAIN_ID_OVERRIDE=9
export ROS_DOMAIN_ID=9
```

4. Save the changes and exit the text editor.

This is required as the camera nodes, the FMU nodes of the drone, and all other nodes need to be on the same ROS Domain ID so that the docker container is able to see all topics associated with the respective nodes. This simplifies the process of communicating with camera and drone nodes.

## Running the Live Code 
The camera node has to be started before the obstacle avoidance nodes are started. 

1. To start the camera node before running the obstacle avoidance nodes, execute the following command:

```bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true pointcloud.ordered_pc:=true
```

- This command will enable the point cloud functionality and ensure that the point cloud data is ordered. Make sure to run this command before starting the obstacle avoidance scripts.

2. To run the obstacle avoidance nodes, 

```bash
python3  "Onboard Codes/ros_pointcloud_publisher.py"
python3  "Onboard Codes/obstacle_avoidance_ws/autonomous_obstacle_avoidance.py"
```
- The `ros_pointcloud_publisher` publishes the filtered point cloud, which filters points that are only in a specific XYZ range. The obstacle is detected based on the average of n points that are close to the drone.

- The `autonomous_obstacle_avoidance` maintains a 1-meter distance threshold from obstacles. This is also able to maintain a straight path if no obstacle is detected. 


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

To run the optimal trajectory tracker between a set of goal points, execute the following command:

```bash
python3 trajectory_tracker.py
```

#### 4.2 Run Obstacle Avoidance 

Video demonstrating obstacle avoidance between a set of goal points: </br>
[![YouTube](http://i.ytimg.com/vi/w9qlEVhUz7A/hqdefault.jpg)](https://www.youtube.com/watch?v=w9qlEVhUz7A)

To run the obstacle avoidance script that maintains a 1-meter distance threshold from obstacles, use the command:

```bash
python3 obstacle_avoidance.py
```

## Notes
- **Obstacle Position:** For the purpose of simulation, the obstacle position is defined halfway between the drones current position and the goal position. Real-time obstacle detection and avoidance is executed in the Onboard Codes scripts
