#!/bin/sh

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

echo $SCRIPTPATH

if ! docker images | grep -q station_docker; then
    sh $SCRIPTPATH/station_docker_build.sh
fi

if ! docker ps | grep -q station_docker; then
    docker run \
        -id \
        --mount type=bind,source=$SCRIPTPATH/../.station_build_cache,target=/20xt_ws/build \
        --mount type=bind,source=$SCRIPTPATH/../src,target=/20xt_ws/src \
        station_docker > /dev/null
fi

docker container exec -it $(docker ps | grep station_docker | awk '{print $NF}') /bin/bash