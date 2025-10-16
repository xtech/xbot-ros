#!/bin/bash
set -e
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

export MY_UID=$(id -u)
export MY_GID=$(id -g)
export MY_USER=xbot

# Create config files
mkdir -p ${SCRIPT_DIR}/.home
touch ${SCRIPT_DIR}/.home/.zsh_history

docker compose down
docker compose up --build -d --wait
exec ./attach.sh
