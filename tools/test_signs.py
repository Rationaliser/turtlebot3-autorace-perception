#!/usr/bin/env python3
"""Place the robot at on-track poses in front of a sign (taken from logged lane-following paths)
and record what the running detect_<mission>_sign node reports on /detect/traffic_sign.
Usage: test_signs.py MISSION OUT_DIR PATH_CSV [PATH_CSV ...]"""
import sys, csv, math, time, subprocess, os, cv2, numpy as np, rclpy
from std_msgs.msg import UInt8
from sensor_msgs.msg import CompressedImage

SIGNS = {  # world poses from turtlebot3_autorace_2020.world
    'intersection': {'intersection': (1.95, -1.0), 'right': (1.08, -0.728), 'left': (0.50, 1.90)},
    'construction': {'construction': (1.0, 0.04)},
    'parking': {'parking': (0.74, 1.95)},
    'level_crossing': {'stop': (-1.35, 1.04)},
    'tunnel': {'tunnel': (-1.544, 0.0)},
}
IDS = {'intersection': ['intersection', 'left', 'right'], 'construction': ['construction'],
       'parking': ['parking'], 'level_crossing': ['stop'], 'tunnel': ['tunnel']}

mission, out = sys.argv[1], sys.argv[2]
path = []
for f in sys.argv[3:]:
    rows = [(float(r['x']), float(r['y'])) for r in csv.DictReader(open(f))]
    for i in range(len(rows) - 2):
        (x0, y0), (x1, y1) = rows[i], rows[i + 2]
        if math.hypot(x1 - x0, y1 - y0) > 0.01:
            path.append((x0, y0, math.atan2(y1 - y0, x1 - x0)))

rclpy.init()
n = rclpy.create_node('sign_tester')
state = {'det': [], 'img': None}
n.create_subscription(UInt8, '/detect/traffic_sign', lambda m: state['det'].append(m.data), 10)
def on_img(m):
    img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
    if img is not None and img.shape[1] > 320:  # frames with the SIFT match overlay are scene+reference side by side
        state['img'] = img
n.create_subscription(CompressedImage, '/detect/image_traffic_sign/compressed', on_img, 10)

def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)

summary = []
for sign, (sx, sy) in SIGNS[mission].items():
    cands = []
    for x, y, yaw in path:
        d = math.hypot(sx - x, sy - y)
        bearing = math.atan2(sy - y, sx - x) - yaw
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
        if 0.25 < d < 0.9 and abs(bearing) < math.radians(30):
            cands.append((d, x, y, yaw))
    cands.sort()
    cands = cands[::max(1, len(cands) // 8)][:8]  # ~8 poses spread over the distance range
    hits, saved = 0, False
    for d, x, y, yaw in cands:
        subprocess.run(['gz', 'model', '-m', 'burger_cam', '-x', str(x), '-y', str(y), '-z', '0.01',
                        '-Y', str(yaw)], check=True)
        spin(0.8)
        state['det'] = []
        state['img'] = None
        spin(2.5)
        names = sorted({IDS[mission][v - 1] for v in state['det'] if 0 < v <= len(IDS[mission])})
        ok = sign in names
        hits += ok
        print(f'{mission:14s} sign={sign:12s} dist={d:.2f}m  detected={names}')
        if ok and not saved and state['img'] is not None:
            cv2.imwrite(os.path.join(out, f'{mission}__{sign}.png'), state['img'])
            saved = True
    summary.append(f'{mission}/{sign}: detected at {hits}/{len(cands)} poses')
print('\n'.join(summary))
rclpy.shutdown()
