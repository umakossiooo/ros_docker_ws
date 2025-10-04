#!/bin/bash
docker run -it --rm \
    --name ros2_jazzy_dev \
    --net=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$HOME/ros_docker_ws:/root/shared" \
    ros2:jazzy-dev bash
