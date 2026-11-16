#!/usr/bin/env python3
"""Phase 2 - Décollage, stationnaire et atterrissage en mode offboard.

Test en simulation : PX4 SITL + Gazebo (gz_x500) + micro-xrce-dds-agent.
Repère PX4 : NED (x nord, y est, z vers le bas), donc l'altitude est -z.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition)

# --- Paramètres de la mission -------------------------------------------
ALTITUDE_M = 5.0        # hauteur du vol stationnaire
HOVER_S = 8.0           # durée du stationnaire
TOLERANCE_M = 0.4       # tolérance pour considérer l'altitude atteinte
PREPARE_CYCLES = 10     # consignes envoyées avant offboard + armement
CYCLE_S = 0.1           # fréquence de la boucle : 10 Hz
CLIMB_TIMEOUT_S = 30.0  # abandon si l'altitude n'est pas atteinte
LAND_TIMEOUT_S = 60.0   # abandon si l'atterrissage n'est pas détecté
GROUND_M = 0.3          # sous cette altitude, le drone est considéré au sol


class OffboardTakeoff(Node):
    def __init__(self):
        super().__init__('offboard_takeoff')

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
        self.state_since = self.now_s()
        self.last_log = 0.0
        self.done = False

        self.create_timer(CYCLE_S, self.tick)
        self.get_logger().info('Nœud démarré, en attente de la position du drone...')

    # --- Utilitaires ------------------------------------------------------
    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def altitude(self):
        return 0.0 if self.position is None else -self.position.z

    def set_state(self, new_state):
        self.get_logger().info(f'État : {self.state} -> {new_state}')
        self.state = new_state
        self.state_since = self.now_s()

    def on_position(self, msg):
        self.position = msg

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

    def command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
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

    # --- Boucle principale (10 Hz) ----------------------------------------
    def tick(self):
        elapsed = self.now_s() - self.state_since

        if self.now_s() - self.last_log >= 1.0:
            self.last_log = self.now_s()
            self.get_logger().info(
                f'[{self.state}] altitude = {self.altitude():.2f} m')

        # Après NAV_LAND, PX4 quitte le mode offboard : on n'envoie plus rien.
        if self.state == 'LAND':
            if elapsed > 3.0 and self.altitude() < GROUND_M:
                self.get_logger().info('Atterrissage terminé.')
                self.done = True
            elif elapsed > LAND_TIMEOUT_S:
                self.get_logger().error('Atterrissage non détecté : arrêt du nœud.')
                self.done = True
            return

        # Le "battement de coeur" offboard doit être envoyé en continu.
        self.publish_offboard_mode()
        self.publish_setpoint(0.0, 0.0, -ALTITUDE_M)

        if self.state == 'PREPARE':
            self.cycles += 1
            if self.cycles >= PREPARE_CYCLES and self.position is not None:
                self.engage_offboard_and_arm()
                self.set_state('CLIMB')

        elif self.state == 'CLIMB':
            if self.altitude() >= ALTITUDE_M - TOLERANCE_M:
                self.set_state('HOVER')
            elif elapsed > CLIMB_TIMEOUT_S:
                self.get_logger().error(
                    'Altitude non atteinte : le drone est-il armé ? '
                    'Regardez QGroundControl et le terminal de PX4.')
                self.done = True

        elif self.state == 'HOVER':
            if elapsed >= HOVER_S:
                self.land()
                self.set_state('LAND')


def main():
    rclpy.init()
    node = OffboardTakeoff()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warn("Interruption : demande d'atterrissage.")
        node.land()
        time.sleep(0.3)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
