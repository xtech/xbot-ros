#!/bin/bash
set -e
export MY_UID=$(id -u)
export MY_GID=$(id -g)
export MY_USER=${USER}
export TERM=xterm-256color
export COLORTERM=truecolor

docker compose exec -it ros2 /ros_entrypoint.sh zsh
