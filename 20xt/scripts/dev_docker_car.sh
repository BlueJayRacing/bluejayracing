#!/bin/sh

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

if ! docker images | grep -q car_docker; then
    sh $SCRIPTPATH/build_docker_car.sh
fi

if ! docker ps | grep -q car_docker; then
    docker run \
        -id \
        --mount type=bind,source=$SCRIPTPATH/../build_cache,target=/20xt_ws/build \
        --mount type=bind,source=$SCRIPTPATH/../src,target=/20xt_ws/src \
        car_docker > /dev/null
fi

docker container exec -it $(docker ps | grep car_docker | awk '{print $NF}') /bin/bash