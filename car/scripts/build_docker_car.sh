#!/bin/sh

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
docker buildx build --platform="arm64" -t bjr_docker_ros -f $SCRIPTPATH/../docker/Dockerfile_car_ros $SCRIPTPATH/..

if docker ps | grep -q car_docker; then
    sh $SCRIPTPATH/clean_docker_car.sh
fi

if ! ls $SCRIPTPATH/.. | grep -q build_cache; then
    mkdir -p $SCRIPTPATH/../build_cache
fi
