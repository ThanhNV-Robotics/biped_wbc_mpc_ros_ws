#!/usr/bin/env bash
#
# Usage:
#
# $ cd ~/your_colcon_ws/src/mujoco_ros2_control/docker
# $ ./launch_docker_image.bash    # Launch the mujoco_ros2_control Docker container
#
# (Cross reference this file with the "run" section of ../.devcontainer/devcontainer.json)
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Allow GUI applications
xhost +SI:localuser:root

# Generate Xauthority file for X11 forwarding
XAUTH=/tmp/.docker.xauth
if [ ! -f "${XAUTH}" ]; then
  touch "${XAUTH}"
  xauth nlist "${DISPLAY}" \
    | sed -e 's/^..../ffff/' \
    | xauth -f "${XAUTH}" nmerge -
  chmod a+r "${XAUTH}"
fi

# Root colcon workspace (contains src/)
HOST_WS="$(realpath "${SCRIPT_DIR}/../../..")"

# Run the container, mounting the entire workspace
docker run --rm -it \
  --name mujoco-ros2-control-dev \
  --net host \
  --privileged \
  -u root \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY="${XAUTH}" \
  -e XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${XAUTH}:${XAUTH}:rw" \
  -v "${HOST_WS}:/mujoco_ros2_control_ws:cached" \
  --workdir /mujoco_ros2_control_ws \
  mujoco-ros2-control:dev \
  bash

echo "Done."

