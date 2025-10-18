FROM ros:jazzy AS base

# Update rosdep package manager for ROS
RUN rosdep update --rosdistro $ROS_DISTRO

# Create a new image layer based on the base image to build a build dependency list (for caching purposes)
FROM base AS build-dep-list

# Build the build dependency-list
RUN --mount=type=bind,source=.,target=/workspace \
    cd /workspace && \
    rosdep install --from-paths src --ignore-src --simulate | \
    sed --expression '1d' | sort | tr -d '\n' | sed --expression 's/  apt-get install//g' > /apt-install_list

# Create a new image layer based on the base image to build a runtime dependency list (for caching purposes)
FROM base AS runtime-dep-list

# Build the runtime dependency-list
RUN --mount=type=bind,source=.,target=/workspace \
    cd /workspace && \
    rosdep install --from-paths src --ignore-src --simulate -t exec | \
    sed --expression '1d' | sort | tr -d '\n' | sed --expression 's/  apt-get install//g' > /apt-install_list


FROM base AS builder

# Copy the list of packages into the new layer (so the next steps can be cached if the list doesn't change)
COPY --from=build-dep-list /apt-install_list /apt-install_list
COPY ./docker/additional-apt-packages.txt /

RUN apt-get update && \
    apt-get install --no-install-recommends --yes $(cat /apt-install_list) $(cat /additional-apt-packages.txt) libasio-dev python3 python3-pip python3-venv && \
    rm -rf /var/lib/apt/lists/*

COPY . /workspace
WORKDIR /workspace
RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build --cmake-args -DFASTDDS_STATISTICS=ON"


FROM base AS runtime
# Copy the list of packages into the new layer (so the next steps can be cached if the list doesn't change)
COPY --from=runtime-dep-list /apt-install_list /apt-install_list
COPY ./docker/additional-apt-packages.txt /
RUN apt-get update && \
    apt-get install --no-install-recommends --yes $(cat /apt-install_list) $(cat /additional-apt-packages.txt) libasio-dev python3 python3-pip python3-venv && \
    rm -rf /var/lib/apt/lists/* && \
    rm /apt-install_list && \
    rm /additional-apt-packages.txt

COPY --from=builder /workspace/install /workspace/install

COPY ./docker/entrypoint.sh /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

COPY ./docker/assets/02-ros.sh /etc/profile.d/02-ros.sh

CMD ["sleep", "infinity"]
