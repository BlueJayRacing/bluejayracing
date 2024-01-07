#!/bin/sh

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
docker build -t car_docker -f $SCRIPTPATH/../docker/Dockerfile_car $SCRIPTPATH

if docker ps | grep -q car_docker; then
    sh $SCRIPTPATH/clean_docker_car.sh
fi

if ! ls $SCRIPTPATH/.. | grep -q build_cache; then
    mkdir -p $SCRIPTPATH/../build_cache
fi