#!/bin/sh

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

echo $SCRIPTPATH
sh $SCRIPTPATH/_car_run_container.sh


docker container exec -it $(docker ps | grep car_docker | awk '{print $NF}') /bin/bash -c "\
    echo 'Launching container and building...'; \
    ./car_proc_build.sh; \
    echo; \
    echo 'executables built, starting applications in background'; \
    echo; \
    ./car_proc_run.sh"