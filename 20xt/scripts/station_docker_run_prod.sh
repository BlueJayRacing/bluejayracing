#!/bin/sh

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

echo $SCRIPTPATH
sh $SCRIPTPATH/_station_run_container.sh


docker container exec -it $(docker ps | grep station_docker | awk '{print $NF}') /bin/bash -c "\
    echo 'Launching container and building...'; \
    ./station_proc_build.sh; \
    echo; \
    echo 'executables built, starting applications in background'; \
    echo; \
    ./station_proc_run.sh"