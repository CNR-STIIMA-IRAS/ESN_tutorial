#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from geometry_msgs.msg import Twist
from esn_msgs.action import PickObject

# import numpy as np

class RobotActionServer(Node):
    def __init__(self):
        super().__init__('robot_action_server_node')

        # Parametri
        self.declare_parameter('motion_duration', 2.0)
        self.motion_duration = self.get_parameter('motion_duration').value

        self.declare_parameter('initial_x', 0.0)
        self.initial_x = self.get_parameter('initial_x').value

        self.declare_parameter('initial_y', 0.0)
        self.initial_y = self.get_parameter('initial_y').value

        self.declare_parameter('action_name', 'pick_object')
        action_name = self.get_parameter('action_name').value

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Action server
        self._action_server = ActionServer(
            self,
            PickObject,
            action_name,
            execute_callback=self.__execute_callback,
            goal_callback=self.__goal_callback,
            cancel_callback=self.__cancel_callback,
        )
        self.robot_busy = False

    # ---- GOAL MANAGEMENT ----
    def __goal_callback(self, goal_request: PickObject.Goal):
        self.get_logger().info(
            f"Received goal pose [{goal_request.pose.header.frame_id}] "
            f"pos=({goal_request.pose.pose.position.x:.2f}, "
            f"{goal_request.pose.pose.position.y:.2f})"
        )

        # q = goal_request.pose.orientation
        # orientation_vec = np.array([q.x, q.y, q.z, q.w])
        # if not np.isclose(np.linalg.norm(orientation_vec), 1.0, atol=1e-6):
        #     self.get_logger().warning("Rejecting goal: quaternion is not normalized.")
        #     return GoalResponse.REJECT

        if self.robot_busy:
            self.get_logger().warning('Rejecting goal: robot is busy.')
            return GoalResponse.REJECT

        self.get_logger().info('Accepting goal.')
        self.robot_busy = True
        return GoalResponse.ACCEPT

    def __cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info("Cancel request received")
        return CancelResponse.ACCEPT

    # ---- EXECUTION ----
    def __execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")

        goal: PickObject.Goal = goal_handle.request
        target_x = float(goal.pose.pose.position.x)
        target_y = float(goal.pose.pose.position.y)

        dx = target_x - self.initial_x
        dy = target_y - self.initial_y

        T = self.motion_duration if self.motion_duration > 1e-6 else 1e-6
        vx = dx / T
        vy = dy / T

        self.get_logger().info(
            f"Computed velocity: vx={vx:.3f}, vy={vy:.3f} to reach in {T:.2f}s"
        )

        rate_hz = 10.0
        dt = 1.0 / rate_hz
        period = dt

        t = 0.0
        cmd = Twist()
        stop = Twist()

        feedback = PickObject.Feedback()
        result = PickObject.Result()

        while t < T and rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.cmd_pub.publish(stop)
                feedback.progress = float(t / T)
                goal_handle.publish_feedback(feedback)

                result.success = False
                goal_handle.canceled()
                self.get_logger().warn(
                    f"Goal canceled at {feedback.progress * 100:.0f}%"
                )
                self.robot_busy = False
                return result

            cmd.linear.x = vx
            cmd.linear.y = vy
            self.cmd_pub.publish(cmd)

            feedback.progress = float(t / T)
            goal_handle.publish_feedback(feedback)

            time.sleep(period)
            t += dt
        # fermo sempre il robot
        self.cmd_pub.publish(stop)
        self.robot_busy = False
        
        if rclpy.ok():
            result.success = True
            goal_handle.succeed()
            self.get_logger().info("Goal succeeded, reached target pose")

            # aggiorno la posizione iniziale
            self.initial_x = target_x
            self.initial_y = target_y

        return result