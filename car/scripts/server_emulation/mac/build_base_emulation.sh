#!/bin/bash
# Determine the absolute directory path of this script.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Build the Docker image for the specified platform.
docker buildx build --platform="linux/arm64" -t bjr_base_ros -f "$SCRIPT_DIR/../../../docker/Dockerfile_base_ros" "$SCRIPT_DIR/../../.."

# If a container from the image is running, clean it up.
if [ -n "$(docker ps --filter "ancestor=bjr_base_ros" --format '{{.ID}}')" ]; then
    "$SCRIPT_DIR/clean.sh"
fi
