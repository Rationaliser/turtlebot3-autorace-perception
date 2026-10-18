#!/bin/bash
# Builds the TurtleBot3 AutoRace simulation workspace used for these results (Ubuntu 22.04, ROS 2 Humble).
# Clones the ROBOTIS packages at the exact commits tested and applies the fixes in patches/.
set -e
WS=${1:-$HOME/turtlebot3_ws}
HERE=$(cd "$(dirname "$0")" && pwd)

sudo apt install -y ros-humble-image-transport ros-humble-cv-bridge ros-humble-vision-opencv \
  python3-opencv libopencv-dev ros-humble-image-pipeline ros-humble-turtlebot3* ros-humble-navigation2 ros-humble-nav2-bringup

mkdir -p "$WS/src" && cd "$WS/src"
clone() {  # repo commit
  [ -d "$(basename "$1" .git)" ] || git clone "$1"
  git -C "$(basename "$1" .git)" checkout -q "$2"
}
clone https://github.com/ROBOTIS-GIT/turtlebot3_autorace.git    a714ca077176ec27952b78266e0ce0ad5281edb5
clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git a35a56c8b04877dc89772b598084d8ce648a9023
git -C turtlebot3_autorace apply --check "$HERE/patches/turtlebot3_autorace.patch" 2>/dev/null \
  && git -C turtlebot3_autorace apply "$HERE/patches/turtlebot3_autorace.patch"

source /opt/ros/humble/setup.bash
cd "$WS" && colcon build --symlink-install --packages-skip turtlebot3_manipulation_gazebo

cat <<MSG

Done. Add to ~/.bashrc (or run in each terminal):
  source /usr/share/gazebo/setup.sh
  source /opt/ros/humble/setup.bash
  source $WS/install/setup.bash
  export GAZEBO_PLUGIN_PATH=$WS/build/turtlebot3_gazebo:\$GAZEBO_PLUGIN_PATH
  export TURTLEBOT3_MODEL=burger_cam
MSG
