#!/usr/bin/env python3
"""Phase 4 - Détection d'une cible rouge dans l'image de la caméra du drone.

Abonné à /drone/camera/image ; publie /drone/cible (geometry_msgs/Point) quand
une cible est vue : x = décalage horizontal (-1 gauche ... +1 droite),
y = décalage vertical (-1 haut ... +1 bas), z = part de l'image occupée (0 à 1).
Enregistre quelques images annotées dans ~/drone_ws/logs/.

Usage : PYTHONNOUSERSITE=1 python3 ~/drone_ws/detection_couleur.py
"""
import csv
import os
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

TOPIC_IMAGE = '/drone/camera/image'
TOPIC_CIBLE = '/drone/cible'
DOSSIER = os.path.expanduser('~/drone_ws/logs')
MIN_AIRE_PX = 40          # en dessous, la tache est ignorée (bruit)
MAX_IMAGES = 20           # nombre maximal d'images annotées enregistrées
INTERVALLE_IMAGES_S = 2.0

# Rouge en HSV (OpenCV : H de 0 à 180). Le rouge est de part et d'autre de 0.
# La valeur minimale basse (40) accepte un rouge sombre, comme sous l'éclairage de Gazebo.
BAS_1, HAUT_1 = (0, 120, 40), (10, 255, 255)
BAS_2, HAUT_2 = (170, 120, 40), (180, 255, 255)


def image_vers_tableau(msg):
    """Convertit un message sensor_msgs/Image (rgb8 ou bgr8) en tableau RGB uint8."""
    donnees = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f'Encodage non géré : {msg.encoding}')
    img = donnees.reshape(msg.height, msg.step // 3, 3)[:, :msg.width, :]
    return img[:, :, ::-1] if msg.encoding == 'bgr8' else img


def detecter_rouge(img_rgb):
    """Retourne la plus grande tache rouge, ou None.

    Résultat : dict avec x, y (centre en pixels), aire (pixels), boite (x, y, l, h),
    dx, dy (centre normalisé entre -1 et 1), part (aire / surface de l'image).
    """
    hsv = cv2.cvtColor(np.ascontiguousarray(img_rgb), cv2.COLOR_RGB2HSV)
    masque = cv2.inRange(hsv, BAS_1, HAUT_1) | cv2.inRange(hsv, BAS_2, HAUT_2)
    masque = cv2.morphologyEx(masque, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(masque, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    aire = cv2.contourArea(contour)
    if aire < MIN_AIRE_PX:
        return None
    x, y, l, h = cv2.boundingRect(contour)
    cx, cy = x + l / 2.0, y + h / 2.0
    hauteur, largeur = masque.shape
    return {
        'x': cx, 'y': cy, 'aire': aire, 'boite': (x, y, l, h),
        'dx': (cx - largeur / 2.0) / (largeur / 2.0),
        'dy': (cy - hauteur / 2.0) / (hauteur / 2.0),
        'part': aire / float(largeur * hauteur),
    }


class DetectionCouleur(Node):
    def __init__(self):
        super().__init__('detection_couleur')
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, TOPIC_IMAGE, self.on_image, qos)
        self.pub = self.create_publisher(Point, TOPIC_CIBLE, 10)
        self.nb_images = 0
        self.nb_detections = 0
        self.nb_enregistrees = 0
        self.dernier_log = 0.0
        self.derniere_sauvegarde = 0.0
        os.makedirs(DOSSIER, exist_ok=True)
        self.debut = self.now_s()
        chemin_csv = os.path.join(DOSSIER, time.strftime('detections_%Y%m%d_%H%M%S.csv'))
        self.fichier_csv = open(chemin_csv, 'w', newline='')
        self.csv = csv.writer(self.fichier_csv)
        self.csv.writerow(['t_s', 'detectee', 'dx', 'dy', 'part'])
        self.get_logger().info(f'Détections enregistrées dans {chemin_csv}')
        self.get_logger().info(f'Détection du rouge sur {TOPIC_IMAGE}...')

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_image(self, msg):
        img = image_vers_tableau(msg)
        res = detecter_rouge(img)
        self.nb_images += 1
        maintenant = self.now_s()
        self.csv.writerow([
            round(maintenant - self.debut, 2), 0 if res is None else 1,
            '' if res is None else round(res['dx'], 3),
            '' if res is None else round(res['dy'], 3),
            '' if res is None else round(res['part'], 5)])

        if res is not None:
            self.nb_detections += 1
            point = Point()
            point.x, point.y, point.z = res['dx'], res['dy'], res['part']
            self.pub.publish(point)
            if (self.nb_enregistrees < MAX_IMAGES
                    and maintenant - self.derniere_sauvegarde >= INTERVALLE_IMAGES_S):
                self.enregistrer(img, res)
                self.derniere_sauvegarde = maintenant

        if maintenant - self.dernier_log >= 1.0:
            self.dernier_log = maintenant
            if res is None:
                self.get_logger().info('Aucune cible rouge.')
            else:
                self.get_logger().info(
                    f"Cible : décalage horizontal {res['dx']:+.2f}, vertical {res['dy']:+.2f}, "
                    f"{res['part'] * 100:.1f} % de l'image "
                    f"({self.nb_detections}/{self.nb_images} images)")

    def enregistrer(self, img_rgb, res):
        annotee = np.ascontiguousarray(img_rgb[:, :, ::-1])  # RGB -> BGR pour OpenCV
        x, y, l, h = res['boite']
        cv2.rectangle(annotee, (x, y), (x + l, y + h), (0, 255, 0), 2)
        cv2.putText(annotee, f"cible {res['part'] * 100:.1f}%", (x, max(12, y - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        self.nb_enregistrees += 1
        chemin = os.path.join(DOSSIER, f'detection_{self.nb_enregistrees:02d}.png')
        cv2.imwrite(chemin, annotee)
        self.get_logger().info(f'Image annotée enregistrée : {chemin}')


def main():
    rclpy.init()
    node = DetectionCouleur()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Fin : {node.nb_detections} détections sur {node.nb_images} images.')
        node.fichier_csv.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
