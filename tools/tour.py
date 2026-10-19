#!/usr/bin/env python3
"""Full-course tour of the TurtleBot3 AutoRace 2020 track.

Drives the robot from the start line to the end of the tunnel and runs each task's detector when
the robot reaches it, opening a window with what the camera sees and detects:

  1. Traffic light   - waits at the start line for the light to go from red to green
  2. Intersection    - intersection / left / right sign recognition
  3. Construction    - construction sign + LiDAR obstacle avoidance (LiDAR window)
  4. Parking         - parking sign recognition
  5. Level crossing  - stop sign, then stops at the barrier until it opens
  6. Tunnel          - tunnel sign, then Nav2 drives through the tunnel (RViz)

Between tasks the robot follows the lanes (lane-detection window). Progress along the course is
tracked against the route recorded in 02_lane_detection/, so tasks trigger at the same places
every run. Started by `./run.sh tour [--from TASK]`, which brings up Gazebo and the camera
pipeline first.

Usage: tour.py LOG_DIR [FIRST_TASK]
"""
import csv
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from rcl_interfaces.msg import Log
from std_msgs.msg import Float64, UInt8
from geometry_msgs.msg import Twist

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_DIR = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, 'logs', 'tour')
FIRST = sys.argv[2] if len(sys.argv) > 2 else 'traffic_light'
os.makedirs(LOG_DIR, exist_ok=True)

TASKS = ['traffic_light', 'intersection', 'construction', 'parking', 'level_crossing', 'tunnel']
TITLES = {'traffic_light': 'Traffic light', 'intersection': 'Intersection', 'construction': 'Construction',
          'parking': 'Parking', 'level_crossing': 'Level crossing', 'tunnel': 'Tunnel'}
LANE_VIEW = '/detect/image_lane/compressed'
SIGN_VIEW = '/detect/image_traffic_sign/compressed'
SIGN_NAMES = {  # value published on /detect/traffic_sign by each mission's sign node
    'intersection': {1: 'intersection', 2: 'left turn', 3: 'right turn'},
    'construction': {1: 'construction'},
    'parking': {1: 'parking'},
    'level_crossing': {1: 'stop'},
    'tunnel': {1: 'tunnel'},
}
# Points on the recorded route (world x, y) where each part of the tour starts or ends.
WAYPOINTS = [
    ('intersection_start', (1.77, -1.55)), ('intersection_end', (1.20, -0.71)),
    ('construction_start', (0.55, 0.27)), ('construction_end', (1.45, 1.77)),
    ('parking_start', (1.45, 1.77)), ('parking_end', (0.55, 1.76)),
    ('level_crossing_start', (-1.75, 1.45)), ('level_look', (-1.60, 1.26)), ('level_hold', (-1.36, 1.24)),
    ('level_end', (-0.55, 1.24)),
    ('tunnel_start', (-1.78, 0.62)), ('tunnel_look', (-1.767, 0.47)), ('tunnel_handoff', (-1.80, -0.05)),
]
START_POSE = (0.8, -1.747, 0.0)       # spawn pose on the start line, facing the traffic light
TUNNEL_ENTRY = (-1.53, -0.28, 0.0)    # world pose of the tunnel map's init_pose (0, 0, 0)
# Poses where a sign was recognised in tools/test_signs.py; the robot pauses there to look.
STOP_SIGN_LOOK = (-1.598, 1.257, -0.284)
TUNNEL_SIGN_LOOK = (-1.768, 0.441, -1.517)
# The barrier detector only works 0.50-0.55 m from the barrier (06_level_crossing/distance_sweep_results.txt).
BARRIER_HOLD = (-1.325, 1.2373, 0.0)


def say(msg=''):
    print(msg, flush=True)


class Procs:
    """Background processes, each in its own session so it can be stopped as a group."""

    def __init__(self):
        self.p = {}

    def running(self, name):
        return name in self.p and self.p[name].poll() is None

    def start(self, name, *cmd):
        self.stop(name)
        log = open(os.path.join(LOG_DIR, f'{name}.log'), 'a')
        self.p[name] = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def stop(self, name):
        proc = self.p.pop(name, None)
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=8)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def stop_all(self):
        for name in list(self.p):
            self.stop(name)


class Tour:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('autorace_tour')
        self.procs = Procs()
        self.route = self.load_route()
        self.wp = self.map_waypoints()
        self.idx = 0
        self.light = (None, 0.0)          # last traffic light colour and when it was seen
        self.barrier_down_at = 0.0        # last time the level-crossing detector saw the barrier down
        self.sign_mission = None
        self.seen = {}                    # task title -> things detected there
        self.nudges = 0
        self.view_topic = None
        self.pub_max_vel = self.node.create_publisher(Float64, '/control/max_vel', 1)
        self.pub_cmd = self.node.create_publisher(Twist, '/cmd_vel', 1)
        self.node.create_subscription(Log, '/rosout', self.on_log, 100)
        self.node.create_subscription(UInt8, '/detect/traffic_sign', self.on_sign, 10)

    # ---------- route and robot pose ----------
    def load_route(self):
        pts = []
        for f in ('lane_following_log.csv', 'lane_following_log_part2.csv'):
            with open(os.path.join(HERE, '02_lane_detection', f)) as fh:
                pts += [(float(r['x']), float(r['y'])) for r in csv.DictReader(fh)]
        route = [pts[0]]
        for p in pts[1:]:
            if math.dist(p, route[-1]) > 0.005:
                route.append(p)
        return route

    def map_waypoints(self):
        out, last = {}, 0
        for name, xy in WAYPOINTS:
            last = min(range(last, len(self.route)), key=lambda i: math.dist(self.route[i], xy))
            out[name] = last
        return out

    def pose(self):
        v = subprocess.run(['gz', 'model', '-m', 'burger_cam', '-p'], capture_output=True, text=True).stdout.split()
        return (float(v[0]), float(v[1]), float(v[5])) if len(v) >= 6 else None

    def update_progress(self):
        p = self.pose()
        if p is None:
            return self.idx
        lo, hi = max(0, self.idx - 5), min(len(self.route), self.idx + 150)
        self.idx = min(range(lo, hi), key=lambda i: math.dist(self.route[i], p[:2]))
        return self.idx

    def place(self, x, y, yaw):
        subprocess.run(['gz', 'model', '-m', 'burger_cam', '-x', str(x), '-y', str(y), '-z', '0.01',
                        '-Y', str(yaw)], capture_output=True)

    def heading_at(self, i):
        a, b = self.route[i], self.route[min(i + 5, len(self.route) - 1)]
        return math.atan2(b[1] - a[1], b[0] - a[0])

    def place_on_route(self, i):
        self.place(*self.route[i], self.heading_at(i))
        self.idx = i

    # ---------- ROS callbacks ----------
    def on_log(self, m):
        if m.name == 'detect_traffic_light' and m.msg.endswith('light detected'):
            colour = m.msg.split()[0]
            if colour != self.light[0]:
                say(f'      light is {colour.upper()}')
            self.light = (colour, time.time())
            self.seen.setdefault('Traffic light', set()).add(colour)
        elif m.name == 'detect_level_crossing' and m.msg in ('slowdown', 'stop'):
            self.barrier_down_at = time.time()

    def on_sign(self, m):
        if self.sign_mission is None:
            return
        name = SIGN_NAMES[self.sign_mission].get(m.data)
        task = TITLES[self.sign_mission]
        if name and name not in self.seen.setdefault(task, set()):
            self.seen[task].add(name)
            say(f'      recognised the {name} sign')

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # ---------- windows, detectors, driving ----------
    def view(self, topic):
        if topic == self.view_topic:
            return
        self.procs.stop('view')
        self.view_topic = topic
        if topic:
            self.procs.start('view', 'ros2', 'run', 'rqt_image_view', 'rqt_image_view', topic)

    def signs(self, mission):
        self.procs.stop('signs')
        self.sign_mission = mission
        if mission:
            self.procs.start('signs', 'ros2', 'launch', 'turtlebot3_autorace_detect', 'detect_sign.launch.py',
                             f'mission:={mission}')

    def max_vel(self, v):
        for _ in range(3):
            self.pub_max_vel.publish(Float64(data=v))
            self.spin(0.05)

    def start_driving(self):
        if not self.procs.running('control_lane'):
            self.procs.start('control_lane', 'ros2', 'launch', 'turtlebot3_autorace_mission',
                             'control_lane.launch.py')
        self.view(LANE_VIEW)
        say('      following the lanes (window: lane detection)')

    def drive_to(self, name):
        """Lane-follow until the robot passes waypoint `name`; nudge it on if it gets stuck."""
        target, last_idx, last_move = self.wp[name], self.idx, time.time()
        while self.update_progress() < target:
            self.spin(0.5)
            if self.idx > last_idx + 2:
                last_idx, last_move = self.idx, time.time()
            elif time.time() - last_move > 25:
                j = min(self.idx + 15, len(self.route) - 1)
                say(f'      robot stuck at ({self.route[self.idx][0]:.2f}, {self.route[self.idx][1]:.2f}) - '
                    'moving it 0.3 m along the course')
                self.place_on_route(j)
                self.nudges += 1
                last_idx, last_move = j, time.time()

    def look_at_sign(self, waypoint, pose, task, sign):
        """Stop at `waypoint`, line up on a pose where `sign` is known to be recognisable, wait up to 8 s."""
        self.drive_to(waypoint)
        self.max_vel(0.0)
        if sign not in self.seen.get(task, set()):
            say(f'      pausing in front of the {sign} sign to look at it')
            self.place(*pose)
            near = range(max(0, self.idx - 50), min(len(self.route), self.idx + 150))
            self.idx = min(near, key=lambda i: math.dist(self.route[i], pose[:2]))
            start = time.time()
            while sign not in self.seen.get(task, set()) and time.time() - start < 8:
                self.spin(0.2)
        self.max_vel(0.1)

    def arrive(self, task, what):
        say()
        say(f'[{TASKS.index(task) + 1}/6] {TITLES[task]}')
        say(f'      {what}')

    # ---------- the six tasks ----------
    def traffic_light(self):
        self.arrive('traffic_light', 'window: camera view with the detected light colour')
        self.procs.start('traffic_light', 'ros2', 'launch', 'turtlebot3_autorace_detect',
                         'detect_traffic_light.launch.py')
        self.view('/detect/image_traffic_light/compressed')
        say('      waiting for the light to turn green')
        start, saw_stop = time.time(), False
        while time.time() - start < 45:
            self.spin(0.2)
            colour, at = self.light
            fresh = time.time() - at < 1.0
            if fresh and colour in ('red', 'yellow'):
                saw_stop = True
            elif fresh and colour == 'green' and saw_stop:
                say('      GREEN - go')
                break
        else:
            say('      no red-to-green change seen in 45 s - going anyway')
        self.procs.stop('traffic_light')
        self.start_driving()

    def intersection(self):
        self.drive_to('intersection_start')
        self.arrive('intersection', 'window: sign recognition (SIFT matches drawn to the reference sign)')
        self.signs('intersection')
        self.view(SIGN_VIEW)
        self.drive_to('intersection_end')
        self.signs(None)
        self.view(LANE_VIEW)

    def construction(self):
        self.drive_to('construction_start')
        self.arrive('construction', 'windows: sign recognition + LiDAR danger zone ("Cluster Points & Danger Zone")')
        self.signs('construction')
        self.view(SIGN_VIEW)
        self.procs.start('avoid_construction', 'ros2', 'run', 'turtlebot3_autorace_mission', 'avoid_construction')
        self.drive_to('construction_end')
        self.procs.stop('avoid_construction')
        with open(os.path.join(LOG_DIR, 'avoid_construction.log'), errors='replace') as fh:
            swerves = fh.read().count('Return turn completed')
        if swerves:
            say(f'      LiDAR saw obstacles in the lane: swerved around {swerves}')
            self.seen.setdefault('Construction', set()).add(f'LiDAR swerves: {swerves}')
        self.signs(None)

    def parking(self):
        self.drive_to('parking_start')
        self.arrive('parking', 'window: sign recognition')
        self.signs('parking')
        self.view(SIGN_VIEW)
        self.drive_to('parking_end')
        self.signs(None)
        self.view(LANE_VIEW)

    def level_crossing(self):
        self.drive_to('level_crossing_start')
        self.arrive('level_crossing', 'window: stop-sign recognition, then the barrier detector')
        self.signs('level_crossing')
        self.view(SIGN_VIEW)
        self.procs.start('level_crossing', 'ros2', 'launch', 'turtlebot3_autorace_detect',
                         'detect_level_crossing.launch.py')
        self.look_at_sign('level_look', STOP_SIGN_LOOK, 'Level crossing', 'stop')
        self.drive_to('level_hold')
        self.max_vel(0.0)
        self.signs(None)
        self.view('/detect/image_level/compressed')
        self.spin(1.0)
        x = self.pose()[0]
        if abs(-0.8 - x - 0.525) > 0.02:
            say(f'      stopped {-0.8 - x:.2f} m before the barrier; lining up at 0.53 m (detector works at 0.50-0.55 m)')
            self.place(*BARRIER_HOLD)
            self.spin(1.0)
        say('      watching the barrier')
        self.barrier_down_at = 0.0
        start, saw_down = time.time(), False
        while time.time() - start < 35:
            self.spin(0.2)
            if time.time() - self.barrier_down_at < 0.6:
                if not saw_down:
                    say('      barrier DOWN - waiting')
                    self.seen.setdefault('Level crossing', set()).add('barrier down')
                saw_down = True
            elif saw_down and time.time() - self.barrier_down_at > 2.0:
                say('      barrier UP - go')
                self.seen['Level crossing'].add('barrier up')
                break
        else:
            say('      barrier state unclear after 35 s - going anyway')
        self.max_vel(0.1)
        self.drive_to('level_end')
        self.procs.stop('level_crossing')
        self.view(LANE_VIEW)

    def tunnel(self):
        self.drive_to('tunnel_start')
        self.arrive('tunnel', 'window: tunnel-sign recognition, then RViz with the Nav2 map and path')
        self.signs('tunnel')
        self.view(SIGN_VIEW)
        self.look_at_sign('tunnel_look', TUNNEL_SIGN_LOOK, 'Tunnel', 'tunnel')
        self.drive_to('tunnel_handoff')
        self.signs(None)
        self.procs.stop('control_lane')
        self.procs.stop('detect_lane')
        self.pub_cmd.publish(Twist())
        self.view(None)
        say('      no lanes inside the tunnel: switching to Nav2 (aligning the robot on the map\'s start pose)')
        self.place(*TUNNEL_ENTRY)
        log_path = os.path.join(LOG_DIR, 'mission_tunnel.log')
        self.procs.start('mission_tunnel', 'ros2', 'launch', 'turtlebot3_autorace_mission', 'mission_tunnel.launch.py')
        start, done = time.time(), False
        while time.time() - start < 180 and not done:
            self.spin(1.0)
            with open(log_path, errors='replace') as fh:
                done = 'Goal succeeded' in fh.read()
        say('      Nav2: goal reached - out of the tunnel' if done else '      Nav2 did not report reaching the goal in 180 s')
        self.seen.setdefault('Tunnel', set()).add('Nav2 goal reached' if done else 'Nav2 goal NOT reached')

    # ---------- the tour ----------
    def run(self):
        tasks = TASKS[TASKS.index(FIRST):]
        say('AutoRace tour: ' + ' -> '.join(TITLES[t].lower() for t in tasks))
        self.procs.start('detect_lane', 'ros2', 'launch', 'turtlebot3_autorace_detect', 'detect_lane.launch.py')
        if FIRST == 'traffic_light':
            self.place(*START_POSE)
        else:
            # start about 0.8 m before the task on the recorded route, already lane following
            self.place_on_route(max(0, self.wp[f'{FIRST}_start'] - 40))
            self.spin(3.0)
            self.start_driving()
        for task in tasks:
            getattr(self, task)()

        say()
        say('Tour finished. What was detected at each task:')
        for task in tasks:
            got = ', '.join(sorted(self.seen.get(TITLES[task], []))) or 'nothing'
            say(f'  {TITLES[task]:15s} {got}')
        if self.nudges:
            say(f'  (the robot got stuck {self.nudges} time(s) and was moved 0.3 m along the course)')
        say('Press Ctrl+C to close everything.')
        while True:
            self.spin(1.0)

    def shutdown(self):
        self.procs.stop_all()
        try:
            self.pub_cmd.publish(Twist())
            rclpy.shutdown()
        except Exception:
            pass


def _interrupt(*_):
    raise KeyboardInterrupt


if __name__ == '__main__':
    if FIRST not in TASKS:
        sys.exit(f'Unknown task {FIRST!r}; use one of: {", ".join(TASKS)}')
    signal.signal(signal.SIGTERM, _interrupt)  # run.sh stops the tour with SIGTERM
    signal.signal(signal.SIGINT, _interrupt)
    tour = Tour()
    try:
        tour.run()
    except KeyboardInterrupt:
        pass
    finally:
        tour.shutdown()
