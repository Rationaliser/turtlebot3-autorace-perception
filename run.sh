#!/bin/bash
# Launch the TurtleBot3 AutoRace 2020 simulation with one command.
#
#   ./run.sh [MODE] [--no-view]
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
#   stop             stop a simulation that is still running (e.g. after a closed terminal)
#
# Camera modes (lane, signs, traffic_light, construction, level_crossing) open a window showing
# what the robot's camera sees with the detections drawn on it. --no-view skips that window.
#
# Ctrl+C stops everything. Logs go to ./logs/<mode>/.
# Set TB3_WS if your workspace is not ~/turtlebot3_ws (see setup.sh).

set -u
HERE=$(cd "$(dirname "$0")" && pwd)
WS=${TB3_WS:-$HOME/turtlebot3_ws}

MODE=sim
case "${1:-}" in -*|"") ;; *) MODE=$1; shift ;; esac
case "$MODE" in
  sim|lane|signs|traffic_light|construction|level_crossing|tunnel|teleop|stop) ;;
  help) set -- --help ;;
  *) echo "Unknown mode '$MODE'. Run ./run.sh --help"; exit 1 ;;
esac
MISSION=""
if [ "$MODE" = "signs" ]; then
  case "${1:-}" in -*|"") MISSION=intersection ;; *) MISSION=$1; shift ;; esac
  case "$MISSION" in intersection|construction|parking|level_crossing|tunnel) ;;
    *) echo "Unknown sign mission '$MISSION'. Use intersection, construction, parking, level_crossing or tunnel."; exit 1 ;; esac
fi
VIEW_ON=1
for a in "$@"; do
  case "$a" in
    -h|--help) sed -n '2,21p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    --no-view) VIEW_ON=0 ;;
    --rqt) ;;  # the camera window now opens by default; kept so old commands still work
    *) echo "Unknown option '$a'. Run ./run.sh --help"; exit 1 ;;
  esac
done

# Everything a run may leave behind, matched on the program a process runs (and, for Python, the
# script it runs) - never on later arguments, so shells or editors that merely mention these paths
# are left alone.
sim_pids() {
  ps -eo pid=,args= | awk -v ws="$WS/install/" -v me="$$" '
    $1 == me { next }
    $2 ~ /(^|\/)(gzserver|gzclient)$/ { print $1; next }
    $2 ~ /python3(\.[0-9]+)?$/ && $3 ~ /\/bin\/ros2$/ && ($4 == "launch" || $4 == "run") &&
      $5 ~ /^(turtlebot3_|gazebo_ros|rqt_image_view|nav2_)/ { print $1; next }
    index($2, ws) == 1 || ($2 ~ /python3(\.[0-9]+)?$/ && index($3, ws) == 1) { print $1; next }
    $2 ~ /^\/opt\/ros\/[^\/]+\/lib\/(rqt_image_view|rviz2|image_transport|rclcpp_components|robot_state_publisher|gazebo_ros|nav2_[^\/]*|turtlebot3_[^\/]*)\// { print $1; next }
    $2 ~ /python3(\.[0-9]+)?$/ && $3 ~ /^\/opt\/ros\/[^\/]+\/lib\/(rqt_image_view|gazebo_ros|turtlebot3_[^\/]*)\// { print $1; next }
  '
}
stop_everything() {
  local pids; pids=$(sim_pids)
  [ -z "$pids" ] && return 0
  kill -INT $pids 2>/dev/null
  for _ in $(seq 1 20); do sleep 0.5; [ -z "$(sim_pids)" ] && return 0; done
  kill -KILL $(sim_pids) 2>/dev/null
  sleep 1
}

if [ "$MODE" = "stop" ]; then
  if [ -z "$(sim_pids)" ]; then echo "Nothing is running."; else stop_everything; echo "Simulation stopped."; fi
  exit 0
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

if [ -z "${DISPLAY:-}" ]; then
  echo "No display found (DISPLAY is empty). Run this from a terminal on the desktop, not over plain SSH."; exit 1
fi
if pgrep -x gzserver >/dev/null; then
  echo "A simulation is already running. Stop it first with:  ./run.sh stop"; exit 1
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
  for _ in $(seq 1 20); do sleep 0.5; [ -z "$(sim_pids)" ] && break; done
  for p in "${PIDS[@]}"; do kill -KILL -- -"$p" 2>/dev/null; done
  stop_everything
  echo "Stopped. Logs: $LOG"
  exit 0
}
trap cleanup INT TERM EXIT

wait_for_topic() {  # topic, timeout_s, what
  local t=0
  printf "  waiting for %s " "$3"
  until ros2 topic list 2>/dev/null | grep -qx "$1"; do
    sleep 2; t=$((t+2)); printf "."
    if [ $t -ge "$2" ]; then echo " timed out after ${2}s (see $LOG)"; return 1; fi
  done
  echo " ok"
}

place_robot() {  # x y yaw  (world frame)
  gz model -m burger_cam -x "$1" -y "$2" -z 0.01 -Y "$3" >/dev/null 2>&1 \
    || echo "  (could not move robot with 'gz model')"
  sleep 1
}

# ---------- launch ----------
echo "Starting AutoRace simulation [mode: $MODE${MISSION:+ $MISSION}]"
echo "  (Gazebo can take up to a minute to open the first time)"
start gazebo ros2 launch turtlebot3_gazebo turtlebot3_autorace_2020.launch.py
wait_for_topic /camera/image_raw 180 "the robot camera" || exit 1

if [ "$MODE" != "tunnel" ] && [ "$MODE" != "teleop" ]; then
  start intrinsic ros2 launch turtlebot3_autorace_camera intrinsic_camera_calibration.launch.py
  start extrinsic ros2 launch turtlebot3_autorace_camera extrinsic_camera_calibration.launch.py
  wait_for_topic /camera/image_projected 60 "the camera pipeline" || exit 1
fi

VIEW=""   # topic shown in the camera window
case "$MODE" in
  sim)
    VIEW=/camera/image_raw ;;
  lane)
    start detect_lane  ros2 launch turtlebot3_autorace_detect detect_lane.launch.py
    sleep 3
    start control_lane ros2 launch turtlebot3_autorace_mission control_lane.launch.py
    VIEW=/detect/image_lane/compressed ;;
  signs)
    start detect_sign ros2 launch turtlebot3_autorace_detect detect_sign.launch.py mission:="$MISSION"
    echo "  drive the robot to the signs in a second terminal with:  ros2 run turtlebot3_teleop teleop_keyboard"
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

if [ -n "$VIEW" ] && [ $VIEW_ON -eq 1 ]; then
  if wait_for_topic "$VIEW" 60 "the detector output"; then
    start camera_view ros2 run rqt_image_view rqt_image_view "$VIEW"
  fi
fi

echo
echo "Running."
[ -n "$VIEW" ] && echo "  Camera window topic: $VIEW  (reopen it with: ros2 run rqt_image_view rqt_image_view $VIEW)"
echo "  Logs: $LOG"
echo "Press Ctrl+C to stop."

GUI_RESTARTS=0
while true; do
  sleep 2
  pgrep -x gzserver >/dev/null || { echo "Gazebo exited."; exit 0; }
  # The Gazebo window (gzclient) sometimes crashes on its own; the simulation keeps running, so reopen it.
  if ! pgrep -x gzclient >/dev/null && [ $GUI_RESTARTS -lt 3 ]; then
    GUI_RESTARTS=$((GUI_RESTARTS+1))
    echo "  Gazebo window closed or crashed - reopening it ($GUI_RESTARTS/3)"
    start gazebo_window_$GUI_RESTARTS ros2 launch gazebo_ros gzclient.launch.py
    sleep 8
  fi
done
