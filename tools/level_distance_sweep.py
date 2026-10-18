#!/usr/bin/env python3
"""Replay detect_level_crossing's blob logic offline on frames captured at several distances
from the barrier, over a full open/close cycle, to see which state the node would output.
Thresholds are read from level.yaml so this reflects what the node actually uses."""
import os, math, time, subprocess, yaml, cv2, numpy as np, rclpy, collections
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from turtlebot3_autorace_detect.detect_level_crossing import (
    fnCalcDistanceDot2Dot, fnArrangeIndexOfPoint, fnCheckLinearity, fnCheckDistanceIsEqual)

Y = yaml.safe_load(open(os.path.expanduser(
    '~/turtlebot3_ws/src/turtlebot3_autorace/turtlebot3_autorace_detect/param/level/level.yaml')))
red = Y['/detect_level_crossing']['ros__parameters']['detect']['level']['red']
LO = np.array([red['hue_l'], red['saturation_l'], red['lightness_l']])
HI = np.array([red['hue_h'], red['saturation_h'], red['lightness_h']])
p = cv2.SimpleBlobDetector_Params()
p.minThreshold, p.maxThreshold = 0, 255
p.filterByArea, p.minArea, p.maxArea = True, 150, 500
p.filterByConvexity, p.minConvexity = True, 0.7
det = cv2.SimpleBlobDetector_create(p)

def state(img):
    mask = cv2.bitwise_not(cv2.inRange(cv2.cvtColor(img, cv2.COLOR_BGR2HSV), LO, HI))
    k = det.detect(cv2.GaussianBlur(mask, (5, 5), 0))
    closed_bar = (cv2.inRange(cv2.cvtColor(img, cv2.COLOR_BGR2HSV), LO, HI)[:, :280] > 0)
    xs = np.nonzero(closed_bar.any(0))[0]; ys = np.nonzero(closed_bar.any(1))[0]
    truth = 'closed' if len(xs) and (xs.max() - xs.min()) > (ys.max() - ys.min()) else 'open'
    if len(k) == 3:
        mx = sum(q.pt[0] for q in k) / 3; my = sum(q.pt[1] for q in k) / 3
        o = fnArrangeIndexOfPoint([fnCalcDistanceDot2Dot(mx, my, q.pt[0], q.pt[1]) for q in k])
        p1 = [int(k[o[0]].pt[0]), int(k[o[0]].pt[1] - 1)]; p2 = [int(k[o[2]].pt[0]), int(k[o[2]].pt[1] - 1)]
        p3 = [int(k[o[1]].pt[0]), int(k[o[1]].pt[1] - 1)]
        dx, dy = p3[0] - p1[0], p3[1] - p1[1]
        if dx == 0 or abs(dy / dx) > 2.0:
            return truth, 'go', len(k)
        if fnCheckLinearity(p1, p2, p3) or fnCheckDistanceIsEqual(p1, p2, p3):
            return truth, ('slowdown' if 25 / fnCalcDistanceDot2Dot(*p1, *p2) > 1.0 else 'stop'), len(k)
        return truth, 'none', len(k)
    return truth, ('go' if len(k) <= 1 else 'none'), len(k)

rclpy.init(); n = rclpy.create_node('lvl_offline'); br = CvBridge(); st = {'m': None}
n.create_subscription(Image, '/camera/image_compensated', lambda m: st.__setitem__('m', m), 1)
bx, y0 = -0.8, 1.2373
for d in [0.60, 0.55, 0.525, 0.50, 0.475, 0.45, 0.425, 0.40, 0.35, 0.30]:
    subprocess.run(['gz', 'model', '-m', 'burger_cam', '-x', str(bx - d), '-y', str(y0), '-z', '0.01', '-Y', '0'], check=True)
    time.sleep(0.8); res = collections.Counter(); e = time.time() + 20.5; nxt = 0
    while time.time() < e:
        rclpy.spin_once(n, timeout_sec=0.05)
        if st['m'] is not None and time.time() >= nxt:
            t, s, nk = state(br.imgmsg_to_cv2(st['m'], 'bgr8')); res[(t, s)] += 1; st['m'] = None; nxt = time.time() + 0.5
    print(f'd={d:.3f} m  ' + '  '.join(f'{t}->{s}:{c}' for (t, s), c in sorted(res.items())))
rclpy.shutdown()
