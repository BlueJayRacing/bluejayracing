#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Check if the image exists; if not, build it.
if ! docker images --format '{{.Repository}}' | grep -iq "bjr_base_ros"; then
    "$SCRIPT_DIR/build.sh"
fi

# Try to get a running container's ID from the image.
CONTAINER_ID=$(docker ps --filter "ancestor=bjr_base_ros" --format '{{.ID}}' | head -n 1)

# If no container is running, start one.
if [ -z "$CONTAINER_ID" ]; then
    CONTAINER_ID=$(docker run -d \
        -v "$SCRIPT_DIR/../../../src:/bjr_ws/src/bjr_packages" \
        --env "DISPLAY=172.26.128.1:0" \
        --env "QT_X11_NO_MITSHM=1" \
        -v "$HOME/.ssh:/home/dock/.ssh" \
        -v "$HOME/.Xauthority:/tmp/.X11-unix:rw" \
        bjr_base_ros)
fi

# Open an interactive bash session in the container.
docker exec -it "$CONTAINER_ID" bash
