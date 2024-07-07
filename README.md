# Real-Time UAV Obstacle Avoidance Using NonLinear Programming
This project showcases real-time obstacle avoidance in UAVs by utilizing the ModalAI Voxl 2 Sentinel Drone with the PX4 flight stack and an Intel Realsense D435i camera to precisely detect obstacles in its environment. The detected object positions are incorporated as constraints in a non-linear programming-based path planning algorithm, allowing the UAV to autonomously generate safe flight paths with a defined distance threshold around these obstacles. To ensure continuous paths, reference velocities and accelerations are calculated at each waypoint. By accounting for room dimensions and drone dynamics as constraints in the optimizer, these paths ensure efficient and secure navigation, enabling the drone to reach its designated destination accurately and reliably.

# PX4 Simulation Codes

This guide provides step-by-step instructions to launch the PX4 simulation, including setting up the necessary environment, starting essential services, and running trajectory and obstacle avoidance scripts.

# Setting up the Simulation 
## Prerequisites
Make sure docker is installed on the host linux pc. 

## Setting Up Docker Container 

1. Pull the latest ubuntu 22.04 image by 

```bash
docker pull ubuntu:22.04
```
2. Use the `Simulation Codes` as the mounting point for your docker container so that you can access the files in docker as well as the host. A docker container can be opened by using the script `Simulation Codes/run_simclassic_docker.sh `. Make sure to edit the script for the right docker image. 

The list of docker images on the host can be seen by using `docker images`.

3. Run the following commands to install git and PX4 sim.
```bash
    apt update && apt install git
    apt-get install sudo # makes using sudo commands copied from internet easier to run 
    git clone https://github.com/PX4/PX4-Autopilot.git -b release/1.14 --recursive
    ./Tools/setup/ubuntu.sh # this installs prerequisties 
    pip uninstall em && pip install empy==3.3.4

```
4. Go into the PX4 folder and run `make px4_sitl gz_x500_depth`. A window that says Gazebo Sim starts up. If it has, then the sim ahs been successfully installed. This will open the drone in a empty world. To open in a simulated world , run ` make px4_sitl gz_x500_depth_baylands` 

5. Install ROS2 humble in the docker image by following instructions from [ROS Docs](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

6. Install the apt version of Gazebo for ROS Humble by following this [link](https://gazebosim.org/docs/latest/ros_installation). 


7. Make a directory in the root of the workspace and clone ros_gz bridge.

```bash
    mkdir -p ros2_gzbridge/src
    cd ros2_gzbridge/src
    git clone https://github.com/gazebosim/ros_gz.git -b humble
    export GZ_VERSION=garden
```

8. Now go back to the root of the workspace and install dependencies for `ros_gz bridge`.Then do colcon build of the same. 

```bash
    cd .. 
    rosdep install -r --from-paths src -i -y --rosdistro humble
    source /opt/ros/humble/setup.bash
    colcon build
```
9. Commit the docker container by getting the name of the docker container by running `docker ps` and then:
```bash
    docker commit name_of_docker_container name_of_image:version
```
The `name_of_image` can be any name as desired, and the same goes for `version`. Now exit the docker container.This is for ensuring the same terminal used to build is not used to isolate build artifacts.

10. Run the script to open the docker container again from step 2, with the required edits to change to respective image name and version from step 9. Source the built ros-gz_bridge:

```bash
    source ros2_gzbridge/install/setup.bash
```
11. Make a folder called models and run the script as given below to source gazebo.
Make sure to edit the path in the script before it is run. 
```bash
    ."/home/s/Downloads/Files/Simulation Codes/set_GZ_SIM_RESOURCE_PATH-2.sh"

```
12. After it is sourced, the command to open PX4 sim can be run. Using the below mentioned script, it will open the sim. 


```bash
    ./Simulation Codes/run_baylands_total-1.sh
```
QGC can be used to take off and control the drone. QGC can be downloaded from this [link](http://qgroundcontrol.com/).
## Notes
- It is recommended to create a folder and always start the docker container from it as the command to run the container will mount the specific directory the script is located. Docker containers typically sandbox themselves from the host system.
- Incase the PX4 sim with baylands does not open , try running ` cd px4-models && python3 simulation-gazebo --world baylands` and ` cd PX4-Autopilot && PX4_GZ_STANDALONE=1 make px4_sitl gz_x500_depth` in seperate terminals from the root folder. of the workspace. 
# Running the Simulation
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

# PX4 Onboard Codes
This code is designed to run on a ModalAI Voxl2-based drone, specifically tested on a Sentinel drone for this project. To ensure optimal performance, connect an Intel Realsense D435i camera to the drone. While a USB 2.0 connection was utilized in this project, it is highly recommended to use a USB 3.0 connection for the camera if available.

## Installing Required Software
Install docker on the Sentinal Drone as given on ModalAI [docs](https://docs.modalai.com/voxl-docker-and-cross-installation/).
Create a docker image with Ubuntu 20.04 and ROS Humble. The Intel RealSense SDK and respective ROS2 wrapper can be installed as explained in the [GitHub](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#installation-on-ubuntu) repository for the RealSense ROS Wrapper. This also has to be installed within the docker container.

## Setting up ROS Domain ID Overide and XRCE_DOMAIN_ID_OVERRIDE
It is essential for the camera nodes, FMU nodes of the drone, and all other nodes to be on the same ROS Domain ID. This ensures that the Docker container can access all topics associated with these nodes, thereby simplifying communication with both the camera and drone nodes.
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

4. Save the changes and exit the text editor

## Step-by-Step Instructions

### Run Camera Node
The camera node has to be started before the obstacle avoidance nodes are started. </br>
To start the camera node, execute the following command:
```bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true pointcloud.ordered_pc:=true
```
This command will enable the point cloud functionality and ensure that the point cloud data is ordered. Make sure to run this command before starting the obstacle avoidance scripts.

### Run Point Cloud Publisher
The `ros_pointcloud_publisher` publishes the filtered XYZ point cloud in a specific range. The obstacle is detected based on the average of n closest points to the drone. </br>
To run the `ros_pointcloud_publisher`, execute the following command:
```bash
python3  "Onboard Codes/ros_pointcloud_publisher.py"
```

### Run Obstacle Avoidance
The `autonomous_obstacle_avoidance` maintains a 1-meter distance threshold from detected obstacles. In the event that no obstacle is detected, the optimizer generates a straight line trajectory to the goal point. </br>
To run the obstacle avoidance nodes, execute the following commands:
```bash
python3  "Onboard Codes/obstacle_avoidance_ws/autonomous_obstacle_avoidance.py"
```
