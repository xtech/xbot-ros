#/etc/profile.d/02-ros.sh
# Source the workspace for login shells
if [ -n "${BASH_VERSION-}" ]; then
  [ -f /workspace/install/setup.bash ] && . /workspace/install/setup.bash
elif [ -n "${ZSH_VERSION-}" ]; then
  [ -f /workspace/install/setup.zsh ] && . /workspace/install/setup.zsh
else
  [ -f /workspace/install/setup.sh ]   && . /workspace/install/setup.sh
fi
