#!/bin/bash
for container in $(docker ps -q --filter "ancestor=bjr_base_ros"); do
    docker rm --force "$container"
done
