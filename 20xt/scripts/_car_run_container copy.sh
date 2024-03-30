SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

echo $SCRIPTPATH

sh $SCRIPTPATH/car_docker_build.sh

mkdir -p $SCRIPTPATH/../.car_build_cache
mkdir -p $SCRIPTPATH/../.car_logs

if ! docker ps | grep -q car_docker; then
    docker run \
        -id \
        --mount type=bind,source=$SCRIPTPATH/../.car_build_cache,target=/20xt_ws/bin \
        --mount type=bind,source=$SCRIPTPATH/../.car_logs,target=/20xt_ws/logs \
        --mount type=bind,source=$SCRIPTPATH/../src,target=/20xt_ws/src \
        -it --device=/dev/ttyS0 \
        car_docker > /dev/null
fi