#!/usr/bin/env python3
"""Enregistre une image de la caméra du drone dans ~/drone_ws/logs/image_drone.png.

Usage : PYTHONNOUSERSITE=1 python3 ~/drone_ws/capture_image.py [topic]
"""
import os
import sys

import numpy as np
import rclpy
from matplotlib.image import imsave
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

TOPIC = '/drone/camera/image'
SORTIE = os.path.expanduser('~/drone_ws/logs/image_drone.png')


def image_vers_tableau(msg):
    """Convertit un message sensor_msgs/Image (rgb8, bgr8, mono8) en tableau uint8."""
    donnees = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if msg.encoding in ('rgb8', 'bgr8'):
        img = donnees.reshape(msg.height, msg.step // 3, 3)[:, :msg.width, :]
        return img[:, :, ::-1] if msg.encoding == 'bgr8' else img
    if msg.encoding == 'mono8':
        return donnees.reshape(msg.height, msg.step)[:, :msg.width]
    raise ValueError(f'Encodage non géré : {msg.encoding}')


class Capture(Node):
    def __init__(self, topic):
        super().__init__('capture_image')
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, topic, self.on_image, qos)
        self.fait = False
        self.get_logger().info(f'En attente d\'une image sur {topic}...')

    def on_image(self, msg):
        img = image_vers_tableau(msg)
        os.makedirs(os.path.dirname(SORTIE), exist_ok=True)
        imsave(SORTIE, img, cmap='gray' if img.ndim == 2 else None)
        self.get_logger().info(
            f'Image {msg.width}x{msg.height} ({msg.encoding}) enregistrée : {SORTIE}')
        self.fait = True


def main():
    rclpy.init()
    node = Capture(sys.argv[1] if len(sys.argv) > 1 else TOPIC)
    try:
        while rclpy.ok() and not node.fait:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
