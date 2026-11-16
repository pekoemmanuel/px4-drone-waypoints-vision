#!/usr/bin/env python3
"""Enregistre la vue de la caméra du drone, annotée, sous forme de GIF.

Chaque image porte l'état du suivi (RECHERCHE, SUIVI...) et, si la cible rouge est
détectée, un cadre vert avec sa taille. L'enregistrement démarre quand le suivi
quitte l'état ATTENTE et s'arrête 10 s après le début de l'atterrissage (ou avec
Ctrl+C). Le GIF est écrit dans ~/drone_ws/logs/suivi_camera.gif.

Usage : PYTHONNOUSERSITE=1 python3 ~/drone_ws/enregistreur_gif.py
"""
import os
import sys

import cv2
import numpy as np
import rclpy
from PIL import Image as ImagePIL
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from detection_couleur import detecter_rouge, image_vers_tableau  # noqa: E402

TOPIC_IMAGE = '/drone/camera/image'
TOPIC_ETAT = '/drone/etat_suivi'
SORTIE = os.path.expanduser('~/drone_ws/logs/suivi_camera.gif')
CADENCE_CAMERA = 10.0     # images/s de la caméra
UNE_IMAGE_SUR = 2         # on garde une image sur deux (5 images/s dans le GIF)
MAX_IMAGES = 300          # nombre maximal d'images du GIF
ECHELLE = 1.0             # 0.75 ou 0.5 pour un GIF plus léger
COULEURS = 64             # nombre de couleurs du GIF
FIN_APRES_S = 10.0        # arrêt 10 s après le début de l'atterrissage


class Enregistreur(Node):
    def __init__(self):
        super().__init__('enregistreur_gif')
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, TOPIC_IMAGE, self.on_image, qos)
        self.create_subscription(String, TOPIC_ETAT, self.on_etat, 10)
        self.etat = None
        self.images = []
        self.compteur = 0
        self.debut = None
        self.t_atterrissage = None
        self.fini = False
        self.derniere_image = None
        self.get_logger().info('En attente du démarrage du suivi (état /drone/etat_suivi)...')

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_etat(self, msg):
        self.etat = msg.data
        if self.etat == 'ATTERRISSAGE' and self.t_atterrissage is None:
            self.t_atterrissage = self.now_s()

    def on_image(self, msg):
        if self.fini or self.etat in (None, 'ATTENTE'):
            return
        if self.debut is None:
            self.debut = self.now_s()
            self.get_logger().info('Enregistrement démarré.')
        self.compteur += 1
        if self.compteur % UNE_IMAGE_SUR:
            return

        rgb = np.ascontiguousarray(image_vers_tableau(msg))
        res = detecter_rouge(rgb)
        bgr = np.ascontiguousarray(rgb[:, :, ::-1])
        if res is not None:
            x, y, l, h = res['boite']
            cv2.rectangle(bgr, (x, y), (x + l, y + h), (0, 255, 0), 2)
            self.texte(bgr, f"cible {res['part'] * 100:.1f}%", (x, max(14, y - 6)), (0, 255, 0))
        self.texte(bgr, f'{self.etat}   t = {self.now_s() - self.debut:.1f} s', (6, 16), (255, 255, 255))

        image = ImagePIL.fromarray(np.ascontiguousarray(bgr[:, :, ::-1]))
        if ECHELLE != 1.0:
            image = image.resize((int(image.width * ECHELLE), int(image.height * ECHELLE)))
        sans_tramage = getattr(ImagePIL, 'Dither', ImagePIL).NONE   # texte net, pas de bruit
        self.images.append(image.convert('P', palette=ImagePIL.ADAPTIVE, colors=COULEURS,
                                         dither=sans_tramage))
        self.derniere_image = self.now_s()

        if len(self.images) >= MAX_IMAGES:
            self.get_logger().info('Nombre maximal d\'images atteint.')
            self.fini = True
        elif self.t_atterrissage is not None and self.now_s() - self.t_atterrissage > FIN_APRES_S:
            self.fini = True

    @staticmethod
    def texte(image, texte, position, couleur):
        police = cv2.FONT_HERSHEY_SIMPLEX
        (largeur, hauteur), base = cv2.getTextSize(texte, police, 0.5, 1)
        x, y = position
        cv2.rectangle(image, (x - 3, y - hauteur - 3), (x + largeur + 3, y + base), (0, 0, 0), -1)
        cv2.putText(image, texte, (x, y), police, 0.5, couleur, 1, cv2.LINE_AA)

    def sauvegarder(self):
        if len(self.images) < 2:
            self.get_logger().warn('Pas assez d\'images enregistrées : aucun GIF écrit.')
            return
        os.makedirs(os.path.dirname(SORTIE), exist_ok=True)
        # Durée de chaque image = temps réel écoulé / nombre d'images (le GIF est lu en temps réel,
        # même si la caméra n'a pas tenu ses 10 images/s).
        duree_s = self.derniere_image - self.debut
        duree_ms = max(20, int(1000 * duree_s / len(self.images)))
        self.images[0].save(SORTIE, save_all=True, append_images=self.images[1:],
                            duration=duree_ms, loop=0, optimize=True)
        taille = os.path.getsize(SORTIE) / 1e6
        self.get_logger().info(
            f'GIF écrit : {SORTIE} ({len(self.images)} images, '
            f'{len(self.images) * duree_ms / 1000:.0f} s, {taille:.1f} Mo)')


def main():
    rclpy.init()
    node = Enregistreur()
    try:
        while rclpy.ok() and not node.fini:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.sauvegarder()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
