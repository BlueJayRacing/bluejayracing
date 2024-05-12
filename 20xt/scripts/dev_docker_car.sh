#!/bin/sh

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

if ! docker images | grep -q car_docker_ros; then
    sh $SCRIPTPATH/build_docker_car.sh
fi

if ! docker ps | grep -q car_docker_ros; then
    docker run \
        -id \
        --mount type=bind,source=$SCRIPTPATH/../src/rsp_baja,target=/20xt_ws/src/rsp_baja \
        -it --device=/dev/ttyAMA0 \
        -it --device=/dev/ttyACM0 \
        -t --device=/dev/i2c-1 \
        car_docker_ros > /dev/null
fi

docker container exec -it $(docker ps | grep car_docker_ros | awk '{print $NF}') /bin/bash
