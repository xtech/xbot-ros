FROM osrf/ros:jazzy-desktop-full AS base
ARG MY_UID
ARG MY_GID
ARG MY_USER

# Install basics
RUN apt-get update && apt-get install -y zsh git sudo gdb wget nano

# Remove the existing ubuntu user if it has the same UID
RUN if id -u ubuntu >/dev/null 2>&1 && [ "$(id -u ubuntu)" = "$MY_UID" ]; then \
        userdel -r ubuntu; \
    fi

# Remove the existing ubuntu group if it has the same GID
RUN if getent group ubuntu >/dev/null 2>&1 && [ "$(getent group ubuntu | cut -d: -f3)" = "$MY_GID" ]; then \
        groupdel ubuntu; \
    fi

# Create a new group and user, assign user to the group, and set its shell to zsh
RUN groupadd -g ${MY_GID} -o ${MY_USER} && \
    useradd -m -u ${MY_UID} -g ${MY_GID} -o -s /bin/zsh ${MY_USER} && yes ${MY_USER} | passwd ${MY_USER} && \
    echo "${MY_USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Update rosdep package manager for ROS
RUN rosdep update --rosdistro $ROS_DISTRO

# Create a new image layer based on the base image for storing dependencies
#FROM base AS dependencies

# Generate a sorted list of apt-get install commands corresponding to the dependencies.
# No actual installation is performed in this stage.
#RUN --mount=type=bind,source=./,target=/tmp/open_mower_ros \
#    cd /tmp/open_mower_ros && \
#    rosdep install --from-paths src --ignore-src --simulate | \
#    sed --expression '1d' | sort | tr -d '\n' | sed --expression 's/  apt-get install//g' > /apt-install_list

# Create another image layer for the development environment
FROM base AS devenv

# Copy the list of packages into the new layer
#COPY --link --from=dependencies /apt-install_list /apt-install_list

# TODO: remove and add actual packet list
RUN touch /apt-install_list

# Update package lists for upgrades or new package installation
RUN apt-get update && \
    apt-get install --no-install-recommends --yes $(cat /apt-install_list) libasio-dev python3 python3-pip python3-venv && \
    rm -rf /var/lib/apt/lists/*

COPY ./entrypoint.sh /entrypoint.sh

# Switch to non-root user
USER $MY_USER
# Install zsh, modify prompt, create link so that the history is persisted
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.2.1/zsh-in-docker.sh)" -- \
    -t robbyrussell && \
    echo "PROMPT=\"[\${COMPOSE_PROJECT_NAME}] \${PROMPT}\"" >> ~/.oh-my-zsh/themes/robbyrussell.zsh-theme && \
    mkdir -p /home/${MY_USER}/.persistent && \
    ln -sf /home/${MY_USER}/.persistent/.zsh_history /home/${MY_USER}/.zsh_history

# Fix the autocomplete
RUN activate-global-python-argcomplete --user && echo "eval \"\$(register-python-argcomplete ros2)\"\\neval \"\$(register-python-argcomplete colcon)\"" >> ~/.zshrc

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]

CMD ["sleep", "infinity"]
