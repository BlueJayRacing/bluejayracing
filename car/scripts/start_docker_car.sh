#!/bin/sh -e

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

if ! docker images | grep -q bjr_docker_ros; then
    sh $SCRIPTPATH/build_docker_car.sh
fi

if ! docker ps | grep -q bjr_docker_ros; then
    docker run \
        -di \
        --network=host \
	    --privileged \
	    -v /dev/bus/usb:/dev/bus/usb \
	    -v /dev/:/dev/ \
        -v ~/Documents:~/Documents \
	    --device-cgroup-rule='c 81:* rmw' \
	    --device-cgroup-rule='c 189:* rmw' \
        --mount type=bind,source=$SCRIPTPATH/../src,target=/bjr_ws/src/bjr_packages \
        --device=/dev/ttyAMA0 \
        --device=/dev/ttyACM0 \
	    --device=/dev/ttyACM1 \
        --device=/dev/i2c-1 \
        bjr_docker_ros > /dev/null
    docker container exec \
        -d $(docker ps | grep bjr_docker_ros | awk '{print $NF}') \
        /bin/bash \
        -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch rpi_launch comp.launch.py"
    docker container exec \
        -d \
        -w /bjr_ws/src/bjr_packages/endurance_bags \
        $(docker ps | grep bjr_docker_ros | awk '{print $NF}') \
        /bin/bash \
        -c 'source /opt/ros/humble/setup.bash && source /bjr_ws/install/setup.bash && ros2 bag record -a --storage-preset-profile resilient'
fi
