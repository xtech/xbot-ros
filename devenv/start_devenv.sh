#!/bin/bash
set -e
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

export MY_UID=$(id -u)
export MY_GID=$(id -g)
export MY_USER=xbot

# Create config files
mkdir -p ${SCRIPT_DIR}/.home
touch ${SCRIPT_DIR}/.home/.zsh_history

# Default values
PEER="xbot"
PROFILE_ARG=""

# Check if etherbridge is the first argument
if [[ "$1" == "etherbridge" ]]; then
    PROFILE_ARG="--profile etherbridge"

    # Check for peer argument
    if [[ "$2" == --peer=* ]]; then
        PEER="${2#*=}"
    fi
fi

export PEER
docker compose down
docker compose $PROFILE_ARG up --build -d --wait
exec ./attach.sh