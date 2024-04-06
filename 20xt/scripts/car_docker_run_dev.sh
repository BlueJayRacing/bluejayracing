#!/bin/sh

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

echo $SCRIPTPATH
sh $SCRIPTPATH/_car_run_container.sh


docker container exec -it $(docker ps | grep car_docker | awk '{print $NF}') /bin/bash