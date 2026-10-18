#!/bin/bash
# Launch the TurtleBot3 AutoRace 2020 simulation with one command.
#
#   ./run.sh [MODE] [--rqt]
#
# MODES
#   sim              Gazebo world + camera pipeline only (default)
#   lane             lane detection + lane following
#   signs MISSION    traffic-sign detection; MISSION = intersection | construction | parking | level_crossing | tunnel
#   traffic_light    traffic-light detection (robot at the start line, facing the light)
#   construction     construction mission: lane following + LiDAR obstacle avoidance
#   level_crossing   level-crossing (barrier) detection, robot placed in front of the barrier
#   tunnel           tunnel mission: Nav2 + AMCL through the tunnel (opens RViz)
#   teleop           drive the robot yourself with the keyboard
#
#   --rqt            also open rqt_image_view on the most useful topic for the mode
#
# Ctrl+C stops everything. Logs go to ./logs/<mode>/.
# Set TB3_WS if your workspace is not ~/turtlebot3_ws (see setup.sh).

set -u
HERE=$(cd "$(dirname "$0")" && pwd)
WS=${TB3_WS:-$HOME/turtlebot3_ws}

MODE=${1:-sim}; shift || true
MISSION=""
if [ "$MODE" = "signs" ]; then MISSION=${1:-intersection}; shift || true; fi
RQT=0
for a in "$@"; do [ "$a" = "--rqt" ] && RQT=1; done

case "$MODE" in
  sim|lane|signs|traffic_light|construction|level_crossing|tunnel|teleop) ;;
  -h|--help|help) sed -n '2,21p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
  *) echo "Unknown mode '$MODE'. Run ./run.sh --help"; exit 1 ;;
esac
if [ -n "$MISSION" ]; then
  case "$MISSION" in intersection|construction|parking|level_crossing|tunnel) ;;
    *) echo "Unknown sign mission '$MISSION'"; exit 1 ;; esac
fi

# ---------- environment ----------
if [ ! -f "$WS/install/setup.bash" ]; then
  echo "Workspace not built at $WS. Run ./setup.sh first (or set TB3_WS)."; exit 1
fi
set +u
source /usr/share/gazebo/setup.sh
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export GAZEBO_PLUGIN_PATH=$WS/build/turtlebot3_gazebo:${GAZEBO_PLUGIN_PATH:-}
export TURTLEBOT3_MODEL=burger_cam

if pgrep -x gzserver >/dev/null; then
  echo "A Gazebo server is already running. Close it first (pkill -x gzserver)."; exit 1
fi

LOG=$HERE/logs/$MODE${MISSION:+_$MISSION}
mkdir -p "$LOG"
PIDS=()

start() {  # name, command...
  local name=$1; shift
  echo "  -> $name"
  setsid "$@" > "$LOG/$name.log" 2>&1 &
  PIDS+=($!)
}

cleanup() {
  trap - INT TERM EXIT
  echo; echo "Shutting down..."
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do kill -INT -- -"${PIDS[i]}" 2>/dev/null; done
  sleep 3
  for p in "${PIDS[@]}"; do kill -KILL -- -"$p" 2>/dev/null; done
  pkill -x gzserver 2>/dev/null; pkill -x gzclient 2>/dev/null
  echo "Stopped. Logs: $LOG"
  exit 0
}
trap cleanup INT TERM EXIT

wait_for_topic() {  # topic, timeout_s
  local t=0
  until ros2 topic list 2>/dev/null | grep -qx "$1"; do
    sleep 1; t=$((t+1))
    if [ $t -ge "$2" ]; then echo "Timed out waiting for $1 (see $LOG)"; exit 1; fi
  done
}

place_robot() {  # x y yaw  (world frame)
  gz model -m burger_cam -x "$1" -y "$2" -z 0.01 -Y "$3" >/dev/null 2>&1 \
    || echo "  (could not move robot with 'gz model')"
  sleep 1
}

# ---------- launch ----------
echo "Starting AutoRace simulation [mode: $MODE${MISSION:+ $MISSION}]"
start gazebo ros2 launch turtlebot3_gazebo turtlebot3_autorace_2020.launch.py
echo "  waiting for the robot camera..."
wait_for_topic /camera/image_raw 120

if [ "$MODE" != "tunnel" ] && [ "$MODE" != "teleop" ]; then
  start intrinsic ros2 launch turtlebot3_autorace_camera intrinsic_camera_calibration.launch.py
  start extrinsic ros2 launch turtlebot3_autorace_camera extrinsic_camera_calibration.launch.py
  wait_for_topic /camera/image_projected 60
fi

VIEW=/camera/image_raw
case "$MODE" in
  lane)
    start detect_lane  ros2 launch turtlebot3_autorace_detect detect_lane.launch.py
    sleep 3
    start control_lane ros2 launch turtlebot3_autorace_mission control_lane.launch.py
    VIEW=/detect/image_lane/compressed ;;
  signs)
    start detect_sign ros2 launch turtlebot3_autorace_detect detect_sign.launch.py mission:="$MISSION"
    echo "  drive the robot to the signs with:  ros2 run turtlebot3_teleop teleop_keyboard"
    VIEW=/detect/image_traffic_sign/compressed ;;
  traffic_light)
    place_robot 0.8 -1.747 0.0
    start detect_traffic_light ros2 launch turtlebot3_autorace_detect detect_traffic_light.launch.py
    VIEW=/detect/image_traffic_light/compressed ;;
  construction)
    place_robot 1.19 0.254 0.0
    start mission_construction ros2 launch turtlebot3_autorace_mission mission_construction.launch.py
    VIEW=/detect/image_lane/compressed ;;
  level_crossing)
    place_robot -1.325 1.2373 0.0
    start detect_level_crossing ros2 launch turtlebot3_autorace_detect detect_level_crossing.launch.py
    VIEW=/detect/image_level/compressed ;;
  tunnel)
    # map (0,0,0) of turtlebot3_autorace_mission/map corresponds to world (-1.53,-0.28,0)
    place_robot -1.53 -0.28 0.0
    start mission_tunnel ros2 launch turtlebot3_autorace_mission mission_tunnel.launch.py ;;
  teleop)
    echo "  Use the keys below to drive. Ctrl+C here stops everything."
    ros2 run turtlebot3_teleop teleop_keyboard
    exit 0 ;;
esac

[ $RQT -eq 1 ] && start rqt ros2 run rqt_image_view rqt_image_view "$VIEW"

echo
echo "Running. Useful topic to view: $VIEW   (rqt -> Plugins > Visualization > Image View)"
echo "Logs: $LOG"
echo "Press Ctrl+C to stop."
while true; do
  sleep 2
  pgrep -x gzserver >/dev/null || { echo "Gazebo exited."; exit 0; }
done
