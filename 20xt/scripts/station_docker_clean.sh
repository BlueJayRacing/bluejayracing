#!/bin/sh

# Kill and delete all currently running docker containers
docker rm --force $(docker ps -q)