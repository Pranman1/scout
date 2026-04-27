#!/usr/bin/env bash
# One-shot install of all dependencies for scout_ws.
#
# Run once on any new machine before `colcon build`. Idempotent;
# re-running is fine.
#
#   chmod +x setup_deps.sh
#   ./setup_deps.sh

set -e

echo "==> Installing system (apt) dependencies"
sudo apt update
sudo apt install -y \
    ros-humble-slam-toolbox \
    ros-humble-nav2-bringup \
    ros-humble-nav2-mppi-controller \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-cv-bridge \
    ros-humble-tf-transformations \
    python3-pip \
    xterm

echo "==> Installing Python dependencies (numpy<2 to keep cv_bridge happy)"
pip install --user -r "$(dirname "$0")/requirements.txt"

echo
echo "Done. Now run:"
echo "  colcon build --symlink-install"
echo "  source install/setup.bash"
