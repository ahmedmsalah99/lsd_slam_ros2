#!/bin/bash

xhost +

if [ "$(docker ps -a | grep lsdslam_ros2)" ]
then
    echo "Container already exists."
    echo "Run 'docker start lsdslam_noros; docker attach lsdslam_noros' to start"
    exit
fi

docker run -it \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --name lsdslam_ros2 \
    -e "TERM=xterm-256color" \
    lsdslam_ros2
