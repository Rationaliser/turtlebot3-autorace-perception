#!/usr/bin/env python3
"""Sweep extrinsic projection params and score lane parallelism in /camera/image_projected.
Score = |lane width at top - lane width at bottom| (px); 0 means perfectly parallel lanes."""
import time, sys, cv2, numpy as np, rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

P = 'camera.extrinsic_camera_calibration.'

class Tuner(Node):
    def __init__(self):
        super().__init__('projection_tuner')
        self.cli = self.create_client(SetParameters, '/camera/image_projection/set_parameters')
        self.cli.wait_for_service()
        self.img, self.br = None, CvBridge()
        self.create_subscription(Image, '/camera/image_projected', self.cb, 1)

    def cb(self, m):
        self.img = self.br.imgmsg_to_cv2(m, 'bgr8')

    def set(self, **kw):
        req = SetParameters.Request()
        for k, v in kw.items():
            req.parameters.append(Parameter(name=P + k, value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER, integer_value=int(v))))
        f = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, f)

    def grab(self):
        time.sleep(0.3)
        self.img = None
        while self.img is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.img = None
        while self.img is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.img

def lane_centers(img, row):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    band = hsv[row - 10:row + 10]
    yellow = cv2.inRange(band, (20, 100, 100), (35, 255, 255))
    white = cv2.inRange(band, (0, 0, 200), (180, 30, 255))
    xs = lambda m: np.nonzero(m)[1]
    y, w = xs(yellow), xs(white)
    if len(y) < 20 or len(w) < 20:
        return None
    return y.mean(), w.mean()

def score(img):
    h = img.shape[0]
    top, bot = lane_centers(img, int(h * 0.15)), lane_centers(img, int(h * 0.85))
    if top is None or bot is None:
        return None, None
    wt, wb = top[1] - top[0], bot[1] - bot[0]
    return abs(wt - wb), (wt, wb)

def main():
    rclpy.init()
    t = Tuner()
    base = dict(top_x=86, top_y=0, bottom_x=169, bottom_y=120)
    results = []
    for top_x in range(66, 111, 4):
        cfg = dict(base, top_x=top_x)
        t.set(**cfg)
        s, w = score(t.grab())
        results.append((s if s is not None else 1e9, cfg, w))
        print(f"top_x={top_x:3d}  width top/bottom={w}  |diff|={s}")
    best = min(results, key=lambda r: r[0])
    print('BEST', best[1], 'diff', best[0])
    t.set(**best[1])
    cv2.imwrite(sys.argv[1], t.grab())
    rclpy.shutdown()

main()
