#!/usr/bin/env python3
"""Log odometry, lane centre, line reliabilities and cmd_vel to CSV for DURATION seconds of wall time.
Usage: log_run.py OUT.csv DURATION"""
import sys, time, csv, rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, UInt8
from geometry_msgs.msg import Twist

rclpy.init()
n = rclpy.create_node('run_logger')
s = dict(x=None, y=None, lane=None, yel=None, whi=None, v=0.0, w=0.0)
def setter(k, f): return lambda m: s.__setitem__(k, f(m))
n.create_subscription(Odometry, '/odom', lambda m: s.update(x=m.pose.pose.position.x, y=m.pose.pose.position.y), 10)
n.create_subscription(Float64, '/detect/lane', setter('lane', lambda m: m.data), 10)
n.create_subscription(UInt8, '/detect/yellow_line_reliability', setter('yel', lambda m: m.data), 10)
n.create_subscription(UInt8, '/detect/white_line_reliability', setter('whi', lambda m: m.data), 10)
n.create_subscription(Twist, '/cmd_vel', lambda m: s.update(v=m.linear.x, w=m.angular.z), 10)
out = csv.writer(open(sys.argv[1], 'w'))
out.writerow(['t', 'x', 'y', 'lane_center', 'yellow_rel', 'white_rel', 'cmd_v', 'cmd_w'])
t0 = time.time(); nxt = t0
while time.time() - t0 < float(sys.argv[2]):
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() >= nxt and s['x'] is not None:
        out.writerow([round(time.time() - t0, 2)] + [s[k] for k in ('x', 'y', 'lane', 'yel', 'whi', 'v', 'w')])
        nxt += 0.5
rclpy.shutdown()
