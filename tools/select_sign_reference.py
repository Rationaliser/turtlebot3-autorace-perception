#!/usr/bin/env python3
"""Choose a SIFT reference image for a traffic sign by leave-one-out over on-track views.
1) place robot at on-path poses facing ahead (sign 0.25-0.9 m ahead, within +-30 deg), grab /camera/image_compensated
2) crop the sign from each view (colour blob above the horizon)
3) for each crop as reference, count how many OTHER views pass the node's test
   (good matches > MIN_MATCH_COUNT and MSE < 50000), keep the best crop.
Usage: select_sign_reference.py SIGN OUT_DIR PATH_CSV [PATH_CSV ...]"""
import sys, os, csv, math, time, subprocess, cv2, numpy as np, rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

CFG = {  # sign xy, colour range (HSV) of its face, node's MIN_MATCH_COUNT
    'construction': ((1.0, 0.04), [((15, 120, 120), (35, 255, 255)), ((160, 80, 120), (180, 255, 255)), ((0, 80, 120), (8, 255, 255))], 8),
    'parking': ((0.74, 1.95), [((85, 120, 120), (100, 255, 255))], 9),
}
sign, out = sys.argv[1], sys.argv[2]
(sx, sy), ranges, min_match = CFG[sign]

path = []
for f in sys.argv[3:]:
    rows = [(float(r['x']), float(r['y'])) for r in csv.DictReader(open(f))]
    for i in range(len(rows) - 2):
        (x0, y0), (x1, y1) = rows[i], rows[i + 2]
        if math.hypot(x1 - x0, y1 - y0) > 0.01:
            path.append((x0, y0, math.atan2(y1 - y0, x1 - x0)))
cands = []
for x, y, yaw in path:
    d = math.hypot(sx - x, sy - y)
    b = math.atan2(sy - y, sx - x) - yaw
    b = math.atan2(math.sin(b), math.cos(b))
    if 0.25 < d < 0.9 and abs(b) < math.radians(30):
        cands.append((d, x, y, yaw))
cands.sort()
cands = cands[::max(1, len(cands) // 12)][:12]

rclpy.init(); n = rclpy.create_node('ref_selector'); br = CvBridge(); st = {'m': None}
n.create_subscription(Image, '/camera/image_compensated', lambda m: st.__setitem__('m', m), 1)
def grab():
    time.sleep(0.8)
    for _ in range(2):
        st['m'] = None
        while st['m'] is None:
            rclpy.spin_once(n, timeout_sec=0.05)
    return br.imgmsg_to_cv2(st['m'], 'bgr8')

views, crops = [], []
for d, x, y, yaw in cands:
    subprocess.run(['gz', 'model', '-m', 'burger_cam', '-x', str(x), '-y', str(y), '-z', '0.01', '-Y', str(yaw)], check=True)
    img = grab(); views.append((d, img))
    os.makedirs(os.path.join(out, "views"), exist_ok=True)
    cv2.imwrite(os.path.join(out, "views", f"{sign}_{d:.2f}.png"), img)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = sum(cv2.inRange(hsv, lo, hi) for lo, hi in ranges).clip(0, 255).astype(np.uint8)
    mask[int(img.shape[0] * 0.55):] = 0
    mask = cv2.dilate(mask, np.ones((3, 3), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        crops.append(None); continue
    bx, by, bw, bh = cv2.boundingRect(max(cnts, key=cv2.contourArea))
    p = 2
    crops.append(img[max(0, by - p):by + bh + p, max(0, bx - p):bx + bw + p] if bw * bh >= 100 else None)
rclpy.shutdown()

sift = cv2.SIFT_create(); fl = cv2.FlannBasedMatcher({'algorithm': 0, 'trees': 5}, {'checks': 50})
def passes(ref, scene):
    kr, dr = sift.detectAndCompute(cv2.cvtColor(ref, cv2.COLOR_BGR2GRAY), None)
    ks, ds = sift.detectAndCompute(scene, None)
    if dr is None or ds is None or len(kr) < 2:
        return False
    good = [m for m, nn in fl.knnMatch(ds, dr, k=2) if m.distance < 0.7 * nn.distance]
    if len(good) <= min_match:
        return False
    a = np.float32([ks[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    b = np.float32([kr[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
    return np.sum((a - b) ** 2) / (a.shape[0] * a.shape[1]) < 50000

orig = cv2.imread(os.path.expanduser(f'~/turtlebot3_ws/src/turtlebot3_autorace/turtlebot3_autorace_detect/image_original_backup/{sign}.png'))
print(f'original reference: passes {sum(passes(orig, v) for _, v in views)}/{len(views)} views')
best = (-1, None, None)
for i, c in enumerate(crops):
    if c is None:
        continue
    score = sum(passes(c, v) for j, (_, v) in enumerate(views) if j != i)
    print(f'crop from view {i} (d={views[i][0]:.2f} m, {c.shape[1]}x{c.shape[0]} px): passes {score}/{len(views) - 1} other views')
    if score > best[0]:
        best = (score, i, c)
os.makedirs(out, exist_ok=True)
cv2.imwrite(os.path.join(out, f'{sign}_selected_reference.png'), best[2])
print(f'SELECTED crop from view {best[1]} ({best[0]}/{len(views) - 1})')
