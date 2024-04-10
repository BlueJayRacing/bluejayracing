#!/bin/sh

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

# Before building, we want to kill any old containers
if docker ps | grep -q car_docker; then
    sh $SCRIPTPATH/car_docker_clean.sh
fi

# Build the image
docker build -t car_docker -f $SCRIPTPATH/../docker/Dockerfile_car $SCRIPTPATH

if ! ls $SCRIPTPATH/.. | grep -q .car_build_cache; then
    mkdir -p $SCRIPTPATH/../.car_build_cache
fi
