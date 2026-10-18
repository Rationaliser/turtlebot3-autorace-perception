#!/usr/bin/env python3
"""Save one frame from each given ROS 2 image topic (raw or /compressed) as PNG.
Usage: snap.py OUT_DIR TOPIC [TOPIC ...]"""
import sys, os, cv2, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

class Snap(Node):
    def __init__(self, out, topics):
        super().__init__('snap')
        self.out, self.left, self.br = out, set(topics), CvBridge()
        for t in topics:
            typ = CompressedImage if t.endswith('compressed') else Image
            for qos in (qos_profile_sensor_data, 10):
                self.create_subscription(typ, t, lambda m, t=t: self.cb(t, m), qos)

    def cb(self, topic, msg):
        if topic not in self.left:
            return
        if isinstance(msg, CompressedImage):
            img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        else:
            img = self.br.imgmsg_to_cv2(msg, 'bgr8' if msg.encoding not in ('mono8', '8UC1') else 'mono8')
        name = topic.strip('/').replace('/', '__') + '.png'
        cv2.imwrite(os.path.join(self.out, name), img)
        self.get_logger().info(f'saved {name}')
        self.left.discard(topic)

def main():
    rclpy.init()
    n = Snap(sys.argv[1], sys.argv[2:])
    end = n.get_clock().now().nanoseconds + 15e9
    while n.left and rclpy.ok() and n.get_clock().now().nanoseconds < end:
        rclpy.spin_once(n, timeout_sec=0.2)
    for t in n.left:
        print('NO MESSAGE:', t)
    rclpy.shutdown()

main()
