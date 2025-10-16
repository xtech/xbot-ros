#!/usr/bin/env bash

set -euo pipefail
# Comma-separated list, e.g. WAIT_IFACES="eth0,wlan0"
IFACES=()
if [[ -n "${WAIT_IFACES-}" ]]; then
  RAW="${WAIT_IFACES//[[:space:]]/}"          # strip spaces
  [[ -n "$RAW" ]] && IFS=',' read -r -a IFACES <<< "$RAW"
fi

for i in "${IFACES[@]}"; do
  # 1) Must EXIST now → otherwise fail immediately
  if ! ip link show "$i" &>/dev/null; then
    echo "ERROR: interface '$i' does not exist. Exiting." >&2
    exit 1
  fi

  # 2) Wait until LINK is UP
  until ip link show "$i" up &>/dev/null; do
    echo "Waiting for $i link UP…"; sleep 0.2
  done

  # 3) Wait until it has an IPv4 address
  until ip -4 -o addr show dev "$i" scope global | grep -q 'inet '; do
    echo "Waiting for $i IPv4 address…"; sleep 0.5
  done

  echo "Ready: $i"
done
source /workspace/install/setup.bash

exec "$@"
