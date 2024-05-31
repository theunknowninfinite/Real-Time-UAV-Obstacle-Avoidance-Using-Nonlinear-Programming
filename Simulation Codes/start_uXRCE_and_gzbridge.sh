#!/bin/bash

source /opt/ros/humble/setup.bash

MicroXRCEAgent udp4 -p 8888 &

export GZ_VERSION=garden
source /opt/ros/humble/setup.bash
# source gz_ros_garden_ws/install/local_setup.bash
source /home/root/PX4-Autopilot/ros2_gzbridge/install/local_setup.bash
 

#ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/home/root/gz_camera.yaml
