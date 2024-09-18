#!/bin/sh

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

if ! docker images | grep -q bjr_docker_ros; then
    sh $SCRIPTPATH/build_docker_car.sh
fi

if ! docker ps | grep -q bjr_docker_ros; then
    docker run \
        -id \
        --network=host \
	--privileged \
	-v /dev/bus/usb:/dev/bus/usb \
	-v /dev/:/dev/ \
	--device-cgroup-rule='c 81:* rmw' \
	--device-cgroup-rule='c 189:* rmw' \
        --mount type=bind,source=$SCRIPTPATH/../src,target=/bjr_ws/src/bjr_packages \
        -it --device=/dev/ttyAMA0 \
        -it --device=/dev/ttyACM0 \
	-it --device=/dev/ttyACM1 \
        -it --device=/dev/i2c-1 \
        bjr_docker_ros > /dev/null
fi

docker container exec -it $(docker ps | grep bjr_docker_ros | awk '{print $NF}') /bin/bash
