SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

echo $SCRIPTPATH

sh $SCRIPTPATH/station_docker_build.sh

mkdir -p $SCRIPTPATH/../.station_build_cache
mkdir -p $SCRIPTPATH/../.station_logs


if ! docker ps | grep -q station_docker; then
    docker run \
        -id \
        --mount type=bind,source=$SCRIPTPATH/../.station_build_cache,target=/20xt_ws/bin \
        --mount type=bind,source=$SCRIPTPATH/../.station_logs,target=/20xt_ws/logs \
        --mount type=bind,source=$SCRIPTPATH/../src,target=/20xt_ws/src \
        --network=host \
        -it --device=/dev/ttyAMA0 \
        station_docker > /dev/null
fi
