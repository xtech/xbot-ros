FROM ros:jazzy AS base

# Update rosdep package manager for ROS
RUN rosdep update --rosdistro $ROS_DISTRO

# Create a new image layer based on the base image to build a dependency list (for caching purposes)
FROM base AS dependency-list

# Generate a sorted list of apt-get install commands corresponding to the dependencies.
# No actual installation is performed in this stage.
RUN --mount=type=bind,source=.,target=/workspace \
    cd /workspace && \
    rosdep install --from-paths src --ignore-src --simulate | \
    sed --expression '1d' | sort | tr -d '\n' | sed --expression 's/  apt-get install//g' > /apt-install_list

# Create another image layer for the actual installation of dependencies
FROM base AS dependencies

# Copy the list of packages into the new layer (so the next steps can be cached if the list doesn't change)
COPY --from=dependency-list /apt-install_list /apt-install_list

COPY ./docker/additional-apt-packages.txt /

# Update package lists for upgrades or new package installation
RUN apt-get update && \
    apt-get install --no-install-recommends --yes $(cat /apt-install_list) $(cat /additional-apt-packages.txt) libasio-dev python3 python3-pip python3-venv && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /workspace


FROM dependencies AS builder
COPY . /workspace
RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

FROM dependencies AS runtime
COPY --from=builder /workspace/install /workspace/install

COPY ./docker/entrypoint.sh /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

CMD ["sleep", "infinity"]
