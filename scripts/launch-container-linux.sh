#!/bin/bash

xhost +

if [ "$(docker ps -a | grep lsdslam_ros2)" ]
then
    echo "Container already exists."
    echo "Run 'docker start lsdslam_ros2; docker attach lsdslam_ros2' to start"
    exit
fi

docker run -it \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --name lsdslam_ros2 \
    -e "TERM=xterm-256color" \
    -p 9000:9000 \
    -p 9002:9002 \
    lsdslam_ros2
