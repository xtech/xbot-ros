#!/bin/bash
set -e
export TERM=xterm-256color
export COLORTERM=truecolor

docker compose exec -it ros2 /entrypoint.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard
