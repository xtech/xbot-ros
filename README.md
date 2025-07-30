# Docker Dev Environment
The `devenv` folder contains a docker compose file which contains a full ROS2 desktop environment with all dependencies to run this project.

Start the devenv by executing `start_devenv.sh`, it will build and start the container. Then run `attach.sh` to attach to a shell in the container.

The project will be mounted in `/workspace`.

# How to use the Docker Devenv with CLion
After starting the devenv once, the resulting Docker image can be used in CLion for development (alternatively, you can use the Remove Development feature and attach to the running devenv).

The steps required are:
- Start the devenv at least once
- Open CLion, add a Docker toolchain.
  - Select the built image (xbot-ros2:latest)
  - Remove the `--entrypoint=` from the Docker command
- In CLion go to CMake and select the newly crated toolchain
- Done
