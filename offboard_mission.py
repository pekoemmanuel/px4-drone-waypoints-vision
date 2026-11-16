#!/usr/bin/env python3
"""Phase 3 - Mission par waypoints en mode offboard (PX4 SITL + Gazebo).

Repère PX4 : NED (x nord, y est, z vers le bas). Altitude 5 m => z = -5.
Le drone suit un carré de 10 m de côté, se pose à son point de départ et
enregistre sa trajectoire dans ~/drone_ws/logs/ (CSV) pour l'analyse.
"""
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition)

# --- Mission : waypoints (x nord, y est, z) en mètres ----------------------
WAYPOINTS = [
    (0.0, 0.0, -5.0),     # décollage
    (10.0, 0.0, -5.0),
    (10.0, 10.0, -5.0),
    (0.0, 10.0, -5.0),
    (0.0, 0.0, -5.0),     # retour au point de départ
]
ARRIVAL_TOL_M = 0.5        # distance pour considérer un waypoint atteint
HOLD_S = 2.0               # pause à chaque waypoint
WAYPOINT_TIMEOUT_S = 40.0  # abandon (atterrissage) si un waypoint n'est pas atteint
PREPARE_CYCLES = 10        # consignes envoyées avant offboard + armement
CYCLE_S = 0.1              # fréquence de la boucle : 10 Hz
LAND_TIMEOUT_S = 60.0
GROUND_M = 0.3             # sous cette altitude, le drone est au sol
DISARM_WAIT_S = 6.0        # attente du désarmement automatique après le contact


class OffboardMission(Node):
    def __init__(self):
        super().__init__('offboard_mission')

        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        self.mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', pub_qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', pub_qos)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.on_position, sub_qos)

        self.position = None
        self.state = 'PREPARE'
        self.cycles = 0
        self.index = 0
        self.yaws = self.compute_yaws()
        self.arrival_errors = []
        self.mission_start = None
        self.state_since = self.now_s()
        self.last_log = 0.0
        self.done = False
        self.touchdown_time = None
        self.log_file, self.log_writer = self.open_log()

        self.create_timer(CYCLE_S, self.tick)
        self.get_logger().info(
            f'Mission de {len(WAYPOINTS)} waypoints, en attente de la position...')

    # --- Utilitaires ------------------------------------------------------
    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def altitude(self):
        return 0.0 if self.position is None else -self.position.z

    def distance_to(self, target):
        if self.position is None:
            return math.inf
        p = self.position
        return math.dist((p.x, p.y, p.z), target)

    def set_state(self, new_state):
        self.get_logger().info(f'État : {self.state} -> {new_state}')
        self.state = new_state
        self.state_since = self.now_s()

    def on_position(self, msg):
        self.position = msg

    @staticmethod
    def compute_yaws():
        # Le drone se tourne vers sa direction de déplacement (0 = nord).
        yaws = [0.0]
        for i in range(1, len(WAYPOINTS)):
            dx = WAYPOINTS[i][0] - WAYPOINTS[i - 1][0]
            dy = WAYPOINTS[i][1] - WAYPOINTS[i - 1][1]
            yaws.append(math.atan2(dy, dx) if math.hypot(dx, dy) > 0.1 else yaws[-1])
        return yaws

    # --- Enregistrement de la trajectoire (CSV) ---------------------------
    def open_log(self):
        folder = os.path.expanduser('~/drone_ws/logs')
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, time.strftime('trajectoire_%Y%m%d_%H%M%S.csv'))
        log_file = open(path, 'w', newline='')
        writer = csv.writer(log_file)
        writer.writerow(['t_s', 'etat', 'waypoint', 'x', 'y', 'z',
                         'cible_x', 'cible_y', 'cible_z'])
        self.get_logger().info(f'Trajectoire enregistrée dans {path}')
        return log_file, writer

    def record(self):
        if self.position is None or self.mission_start is None:
            return
        p = self.position
        tx, ty, tz = WAYPOINTS[self.index]
        self.log_writer.writerow([
            round(self.now_s() - self.mission_start, 2), self.state,
            self.index + 1, round(p.x, 3), round(p.y, 3), round(p.z, 3),
            tx, ty, tz])

    def close_log(self):
        if not self.log_file.closed:
            self.log_file.close()

    # --- Messages vers PX4 ------------------------------------------------
    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.now_us()
        self.mode_pub.publish(msg)

    def publish_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [math.nan] * 3
        msg.acceleration = [math.nan] * 3
        msg.jerk = [math.nan] * 3
        msg.yaw = float(yaw)
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

    def engage_offboard_and_arm(self):
        # param1 = 1 (mode personnalisé), param2 = 6 (offboard)
        self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def land(self):
        self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def summary(self):
        if self.arrival_errors:
            mean = sum(self.arrival_errors) / len(self.arrival_errors)
            self.get_logger().info(
                f'Waypoints atteints : {len(self.arrival_errors)}/{len(WAYPOINTS)}, '
                f'erreur moyenne {mean:.2f} m, maximale {max(self.arrival_errors):.2f} m')

    # --- Boucle principale (10 Hz) ----------------------------------------
    def tick(self):
        elapsed = self.now_s() - self.state_since

        if self.now_s() - self.last_log >= 1.0:
            self.last_log = self.now_s()
            p = self.position
            where = 'position inconnue' if p is None else \
                f'x={p.x:.1f} y={p.y:.1f} alt={-p.z:.1f} m'
            self.get_logger().info(
                f'[{self.state}] waypoint {self.index + 1}/{len(WAYPOINTS)} - {where}')

        self.record()

        # Après NAV_LAND, PX4 quitte le mode offboard : on n'envoie plus rien.
        if self.state == 'LAND':
            if elapsed > 3.0 and self.altitude() < GROUND_M:
                if self.touchdown_time is None:
                    self.touchdown_time = self.now_s()
                    self.get_logger().info('Contact avec le sol : attente du désarmement...')
                elif self.now_s() - self.touchdown_time >= DISARM_WAIT_S:
                    self.get_logger().info('Atterrissage terminé.')
                    # Plus de consignes offboard : retour en mode Hold
                    self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0)
                    self.summary()
                    self.done = True
            elif elapsed > LAND_TIMEOUT_S:
                self.get_logger().error('Atterrissage non détecté : arrêt du nœud.')
                self.done = True
            return

        target = WAYPOINTS[self.index]
        self.publish_offboard_mode()
        self.publish_setpoint(*target, yaw=self.yaws[self.index])

        if self.state == 'PREPARE':
            self.cycles += 1
            if self.cycles >= PREPARE_CYCLES and self.position is not None:
                self.engage_offboard_and_arm()
                self.mission_start = self.now_s()
                self.set_state('GOTO')

        elif self.state == 'GOTO':
            error = self.distance_to(target)
            if error <= ARRIVAL_TOL_M:
                self.arrival_errors.append(error)
                self.get_logger().info(
                    f'Waypoint {self.index + 1}/{len(WAYPOINTS)} atteint en '
                    f'{elapsed:.1f} s (erreur {error:.2f} m)')
                self.set_state('HOLD')
            elif elapsed > WAYPOINT_TIMEOUT_S:
                self.get_logger().error(
                    f'Waypoint {self.index + 1} non atteint : atterrissage. '
                    'Le drone est-il armé ? Regardez QGroundControl.')
                self.land()
                self.set_state('LAND')

        elif self.state == 'HOLD':
            if elapsed >= HOLD_S:
                if self.index + 1 < len(WAYPOINTS):
                    self.index += 1
                    self.set_state('GOTO')
                else:
                    self.land()
                    self.set_state('LAND')


def main():
    rclpy.init()
    node = OffboardMission()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warn("Interruption : demande d'atterrissage.")
        node.land()
        time.sleep(0.3)
    finally:
        time.sleep(0.3)  # laisse partir la dernière commande
        node.close_log()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
