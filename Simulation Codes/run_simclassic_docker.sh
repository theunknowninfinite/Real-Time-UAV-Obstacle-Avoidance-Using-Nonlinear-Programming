#!/bin/bash

docker run --rm -it --net=host --ipc=host --pid=host --privileged -v /dev/shm:/dev/shm -e DISPLAY=$DISPLAY -v /dev/input:/dev/input:rw -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v $(pwd)/:/home/root:rw -w /home/root --name=ros2_autonomy u20_ros2_sim:v1 /bin/bash -l

