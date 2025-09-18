#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from geometry_msgs.msg import Twist
from esn_msgs.action import PickObject   # deve contenere goal.pose (geometry_msgs/Pose)

import numpy as np


class RobotActionServer(Node):
    def __init__(self):
        super().__init__('robot_action_server')

        # Parametro: velocità nominale (m/s)
        self.declare_parameter('nominal_speed', 0.20)  # default 0.2 m/s

        # Publisher cmd_vel
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Action server
        self._action_server = ActionServer(
            self,
            PickObject,
            'pick_object',
            execute_callback=self.__execute_callback,
            goal_callback=self.__goal_callback,
            handle_accepted_callback=self.__handle_accepted_callback,
            cancel_callback=self.__cancel_callback,
        )

        self.robot_busy = False

    # ---- GOAL MANAGEMENT ----
    def __goal_callback(self, goal_request: PickObject.Goal):
        # Verifica normalizzazione quaternione
        q = goal_request.pose.orientation
        orientation_vec = np.array([q.x, q.y, q.z, q.w])
        if not np.isclose(np.linalg.norm(orientation_vec), 1.0, atol=1e-6):
            self.get_logger().warning("Rejecting goal: quaternion is not normalized.")
            return GoalResponse.REJECT

        if self.robot_busy:
            self.get_logger().warning('Rejecting goal: robot is busy.')
            return GoalResponse.REJECT

        self.get_logger().info('Accepting goal.')
        self.robot_busy = True
        return GoalResponse.ACCEPT

    def __handle_accepted_callback(self, goal_handle):
        # In rclpy 1.x l'esecuzione parte già in un thread dedicato:
        # qui possiamo solo loggare o preparare risorse.
        self.get_logger().debug('Goal accepted, preparing execution thread...')

    def __cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info(f"Cancel requested: [{goal_handle.goal_id}]")
        return CancelResponse.ACCEPT

    # ---- EXECUTION ----
    def __execute_callback(self, goal_handle):
        self.get_logger().info('Starting motion towards goal pose (x,y).')

        # Preleva target e velocità
        goal: PickObject.Goal = goal_handle.request
        target_x = float(goal.pose.position.x)
        target_y = float(goal.pose.position.y)
        speed = float(self.get_parameter('nominal_speed').get_parameter_value().double_value)

        # Evita velocità nulla o negativa
        if speed <= 0.0:
            self.get_logger().warn('nominal_speed <= 0, imposto 0.2 m/s')
            speed = 0.2

        # Distanza e durata
        distance = math.hypot(target_x, target_y)
        duration = distance / speed if distance > 0.0 else 0.0

        # Versore di direzione nel piano
        vx = (target_x / distance) * speed if distance > 1e-9 else 0.0
        vy = (target_y / distance) * speed if distance > 1e-9 else 0.0

        twist_cmd = Twist()
        # Pubblicazione lineare nel frame di base: qui assumiamo vx->x, vy->y (holonomic).
        # Se il robot non è olonomo, impostare solo twist.linear.x e gestire yaw a parte.
        twist_cmd.linear.x = vx
        twist_cmd.linear.y = vy
        twist_cmd.angular.z = 0.0

        # Loop temporizzato (bloccante) con controllo cancel
        start = time.monotonic()
        period = 0.05  # 20 Hz
        try:
            while True:
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Goal canceled by client.')
                    self.__stop_robot()
                    goal_handle.canceled()
                    self.robot_busy = False
                    return self.__make_result(success=False, message='Canceled')

                now = time.monotonic()
                elapsed = now - start
                if elapsed >= duration:
                    break

                self.cmd_pub.publish(twist_cmd)
                time.sleep(period)
        finally:
            # assicurati che il robot si fermi
            self.__stop_robot()

        self.get_logger().info('Target reached (time-based straight-line).')
        goal_handle.succeed()
        self.robot_busy = False
        return self.__make_result(success=True, message='Reached by time-based cmd_vel')

    # ---- Utility ----
    def __stop_robot(self):
        stop = Twist()
        self.cmd_pub.publish(stop)  # invia almeno un pacchetto a 0
        # piccola pausa per garantire che arrivi
        time.sleep(0.05)
        self.cmd_pub.publish(stop)

    def __make_result(self, success: bool, message: str):
        result = PickObject.Result()
        # imposta campi se esistono
        if hasattr(result, 'success'):
            result.success = success
        if hasattr(result, 'message'):
            result.message = message
        return result


def main(args=None):
    rclpy.init(args=args)
    node = RobotActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
