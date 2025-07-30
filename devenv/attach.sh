#!/bin/bash
set -e
export TERM=xterm-256color
export COLORTERM=truecolor

docker compose exec -it ros2 /ros_entrypoint.sh zsh
