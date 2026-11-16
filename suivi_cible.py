#!/usr/bin/env python3
"""Phase 5 - Suivi de cible en boucle fermée (PX4 SITL + Gazebo + ROS 2).

Le drone décolle, cherche la cible rouge en tournant sur lui-même, la centre dans
l'image (vitesse de lacet proportionnelle au décalage horizontal), s'en approche
jusqu'à une distance d'arrêt estimée par la taille de la cible dans l'image, la
garde quelques secondes, puis atterrit.

Entrées : /drone/cible (geometry_msgs/Point : x = décalage horizontal -1..+1,
          z = part de l'image occupée) et la position locale de PX4.
Sorties : consignes de vitesse PX4 (mode offboard), /drone/etat_suivi (std_msgs/String)
          et un journal CSV ~/drone_ws/logs/suivi_*.csv.

Repère PX4 : NED (x nord, y est, z vers le bas). Le cap est mesuré depuis le nord,
dans le sens horaire : une cible à droite de l'image demande un lacet positif.
"""
import csv
import math
import os
import time

import rclpy
from geometry_msgs.msg import Point
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition)
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import String

# --- Paramètres --------------------------------------------------------------
ALTITUDE_M = 3.0            # altitude de vol
VITESSE_MAX = 1.5           # vitesse d'approche maximale (m/s)
GAIN_LACET = 0.9            # vitesse de lacet (rad/s) par unité de décalage horizontal
LACET_MAX = 0.5             # vitesse de lacet maximale (rad/s)
VITESSE_RECHERCHE = 0.35    # vitesse de rotation pendant la recherche (rad/s)
SENS_RECHERCHE = -1.0       # -1 : antihoraire (le drone démarre vers l'est, la cible est au nord)
PART_ARRET = 0.0045         # part de l'image visée : environ 12 m (à calibrer)
ARRIVEE = 0.93              # arrivée déclarée à 93 % de cette part (évite de ralentir sans fin)
VITESSE_MIN = 0.3           # vitesse minimale tant que l'arrivée n'est pas déclarée (m/s)
ALIGNE_MAX = 0.35           # |décalage| maximal pour avancer
ALIGNE_ARRET = 0.15         # |décalage| maximal pour déclarer l'arrivée
TENUE_S = 5.0               # durée de maintien devant la cible
PERTE_S = 2.0               # cible perdue plus longtemps que ça : retour en recherche
DUREE_MAX_S = 120.0         # durée maximale de la mission
DISTANCE_MAX_M = 40.0       # distance maximale au point de départ
TOUR_MAX_RAD = 2 * math.pi * 1.1   # rotation maximale sans trouver la cible
CYCLES_PREPARATION = 10
CYCLE_S = 0.1               # boucle à 10 Hz
VISIBLE_S = 0.35            # une cible vue il y a moins de 0,35 s est considérée visible
ATTENTE_DESARMEMENT_S = 6.0
SOL_M = 0.3


def borne(valeur, bas, haut):
    return max(bas, min(haut, valeur))


class Suivi:
    """Machine à états du suivi, sans ROS (testable seule).

    États : ATTENTE, DECOLLAGE, RECHERCHE, SUIVI, MAINTIEN, ATTERRISSAGE.
    """

    def __init__(self):
        self.etat = 'ATTENTE'
        self.depuis = 0.0
        self.debut = None
        self.vu_consecutif = 0
        self.dernier_dx = 0.0
        self.perdu_depuis = None
        self.rotation = 0.0
        self.sens = SENS_RECHERCHE

    def changer(self, etat, t):
        self.etat = etat
        self.depuis = t

    def demarrer(self, t):
        self.debut = t
        self.changer('DECOLLAGE', t)

    def commande(self, t, pos, cible, dt):
        """pos = (x, y, z, cap) ; cible = (vue, dx, part).

        Retourne (vx, vy, vz, vitesse_de_lacet) en repère NED."""
        x, y, z, cap = pos
        vue, dx, part = cible
        vx = vy = vlacet = 0.0
        vz = borne(1.0 * (-ALTITUDE_M - z), -1.0, 0.7)   # maintien d'altitude

        if self.etat == 'ATTENTE' or self.etat == 'ATTERRISSAGE':
            return 0.0, 0.0, 0.0, 0.0

        # Sécurités globales
        if self.debut is not None and t - self.debut > DUREE_MAX_S:
            self.changer('ATTERRISSAGE', t)
            return 0.0, 0.0, 0.0, 0.0
        if math.hypot(x, y) > DISTANCE_MAX_M:
            self.changer('ATTERRISSAGE', t)
            return 0.0, 0.0, 0.0, 0.0

        if self.etat == 'DECOLLAGE':
            if -z >= ALTITUDE_M - 0.4:
                self.changer('RECHERCHE', t)
                self.vu_consecutif = 0
                self.rotation = 0.0
            elif t - self.depuis > 20.0:
                self.changer('ATTERRISSAGE', t)
                return 0.0, 0.0, 0.0, 0.0

        elif self.etat == 'RECHERCHE':
            vlacet = self.sens * VITESSE_RECHERCHE
            self.rotation += abs(vlacet) * dt
            self.vu_consecutif = self.vu_consecutif + 1 if vue else 0
            if self.vu_consecutif >= 3:
                self.changer('SUIVI', t)
                self.perdu_depuis = None
            elif self.rotation > TOUR_MAX_RAD:
                self.changer('ATTERRISSAGE', t)
                return 0.0, 0.0, 0.0, 0.0

        elif self.etat == 'SUIVI':
            if vue:
                self.dernier_dx = dx
                self.perdu_depuis = None
                vlacet = borne(GAIN_LACET * dx, -LACET_MAX, LACET_MAX)
                if abs(dx) < ALIGNE_MAX and part < ARRIVEE * PART_ARRET:
                    v = max(VITESSE_MIN,
                            VITESSE_MAX * borne((PART_ARRET - part) / PART_ARRET, 0.0, 1.0))
                    vx, vy = v * math.cos(cap), v * math.sin(cap)
                if part >= ARRIVEE * PART_ARRET and abs(dx) < ALIGNE_ARRET:
                    self.changer('MAINTIEN', t)
            else:
                if self.perdu_depuis is None:
                    self.perdu_depuis = t
                elif t - self.perdu_depuis > PERTE_S:
                    if self.dernier_dx != 0.0:
                        self.sens = math.copysign(1.0, self.dernier_dx)
                    self.rotation = 0.0
                    self.vu_consecutif = 0
                    self.changer('RECHERCHE', t)

        elif self.etat == 'MAINTIEN':
            if vue:
                vlacet = borne(GAIN_LACET * dx, -LACET_MAX, LACET_MAX)
                if part < 0.5 * PART_ARRET:
                    self.changer('SUIVI', t)
            if t - self.depuis >= TENUE_S:
                self.changer('ATTERRISSAGE', t)
                return 0.0, 0.0, 0.0, 0.0

        return vx, vy, vz, vlacet


class NoeudSuivi(Node):
    def __init__(self):
        super().__init__('suivi_cible')
        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', pub_qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', pub_qos)
        self.etat_pub = self.create_publisher(String, '/drone/etat_suivi', 10)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.on_position, sub_qos)
        self.create_subscription(Point, '/drone/cible', self.on_cible, 10)

        self.suivi = Suivi()
        self.position = None
        self.cible = (0.0, 0.0)          # (dx, part)
        self.cible_t = None
        self.cycles = 0
        self.atterrissage_envoye = False
        self.contact_sol = None
        self.done = False
        self.dernier_log = 0.0
        self.debut_csv = self.now_s()
        dossier = os.path.expanduser('~/drone_ws/logs')
        os.makedirs(dossier, exist_ok=True)
        chemin = os.path.join(dossier, time.strftime('suivi_%Y%m%d_%H%M%S.csv'))
        self.fichier_csv = open(chemin, 'w', newline='')
        self.csv = csv.writer(self.fichier_csv)
        self.csv.writerow(['t_s', 'etat', 'x', 'y', 'z', 'cap_deg', 'vue', 'dx', 'part',
                           'vx', 'vy', 'vz', 'vlacet'])
        self.get_logger().info(f'Journal du suivi : {chemin}')
        self.create_timer(CYCLE_S, self.tick)

    # --- Utilitaires -------------------------------------------------------
    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def on_position(self, msg):
        self.position = msg

    def on_cible(self, msg):
        self.cible = (msg.x, msg.z)
        self.cible_t = self.now_s()

    # --- Messages vers PX4 ---------------------------------------------------
    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.now_us()
        self.mode_pub.publish(msg)

    def publish_vitesse(self, vx, vy, vz, vlacet):
        msg = TrajectorySetpoint()
        msg.position = [math.nan] * 3
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.acceleration = [math.nan] * 3
        msg.jerk = [math.nan] * 3
        msg.yaw = math.nan
        msg.yawspeed = float(vlacet)
        msg.timestamp = self.now_us()
        self.setpoint_pub.publish(msg)

    def command(self, command, param1=0.0, param2=0.0, param3=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.now_us()
        self.command_pub.publish(msg)

    def land(self):
        self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    # --- Boucle principale (10 Hz) -------------------------------------------
    def tick(self):
        t = self.now_s()
        etat_msg = String()
        etat_msg.data = self.suivi.etat
        self.etat_pub.publish(etat_msg)

        # Après NAV_LAND, PX4 quitte le mode offboard : on n'envoie plus rien.
        if self.atterrissage_envoye:
            if self.position is not None and -self.position.z < SOL_M:
                if self.contact_sol is None:
                    self.contact_sol = t
                    self.get_logger().info('Contact avec le sol : attente du désarmement...')
                elif t - self.contact_sol >= ATTENTE_DESARMEMENT_S:
                    # Retour en mode Hold (plus de consignes offboard)
                    self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0)
                    self.get_logger().info('Atterrissage terminé.')
                    self.done = True
            return

        self.publish_offboard_mode()
        if self.position is None:
            self.publish_vitesse(0.0, 0.0, 0.0, 0.0)
            return

        p = self.position
        vue = self.cible_t is not None and (t - self.cible_t) < VISIBLE_S
        dx, part = self.cible if vue else (0.0, 0.0)

        if self.suivi.etat == 'ATTENTE':
            self.cycles += 1
            if self.cycles >= CYCLES_PREPARATION:
                # param1 = 1 (mode personnalisé), param2 = 6 (offboard)
                self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.suivi.demarrer(t)

        avant = self.suivi.etat
        vx, vy, vz, vlacet = self.suivi.commande(
            t, (p.x, p.y, p.z, p.heading), (vue, dx, part), CYCLE_S)
        if self.suivi.etat != avant:
            self.get_logger().info(f'État : {avant} -> {self.suivi.etat}')

        if self.suivi.etat == 'ATTERRISSAGE':
            self.land()
            self.atterrissage_envoye = True
            return

        self.publish_vitesse(vx, vy, vz, vlacet)
        self.csv.writerow([
            round(t - self.debut_csv, 2), self.suivi.etat, round(p.x, 2), round(p.y, 2),
            round(p.z, 2), round(math.degrees(p.heading), 1), int(vue),
            round(dx, 3), round(part, 5), round(vx, 2), round(vy, 2), round(vz, 2),
            round(vlacet, 3)])

        if t - self.dernier_log >= 1.0:
            self.dernier_log = t
            cible = f'dx={dx:+.2f} taille={part * 100:.2f} %' if vue else 'cible non vue'
            self.get_logger().info(
                f'[{self.suivi.etat}] x={p.x:.1f} y={p.y:.1f} alt={-p.z:.1f} m '
                f'cap={math.degrees(p.heading):.0f}° - {cible}')


def main():
    rclpy.init()
    node = NoeudSuivi()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warn("Interruption : demande d'atterrissage.")
        node.land()
        time.sleep(0.3)
    finally:
        time.sleep(0.3)  # laisse partir la dernière commande
        node.fichier_csv.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
