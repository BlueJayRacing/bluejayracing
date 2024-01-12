#!/bin/sh

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

# Before building, we want to kill any old containers
if docker ps | grep -q station_docker; then
    sh $SCRIPTPATH/station_docker_clean.sh
fi

# Build the image
docker build -t station_docker -f $SCRIPTPATH/../docker/Dockerfile_station $SCRIPTPATH

if ! ls $SCRIPTPATH/.. | grep -q .station_build_cache; then
    mkdir -p $SCRIPTPATH/../.station_build_cache
fi
