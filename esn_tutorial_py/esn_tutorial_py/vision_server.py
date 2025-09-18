#!/usr/bin/env python3
import random
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from esn_msgs.srv import DetectObject


class VisionSystem(Node):
    def __init__(self):
        super().__init__('vision_srv_server_node')

        # RNG (seed di default dal sistema)
        self.__rng = random.Random()

        # Parametri
        self.declare_parameter('service_name', 'detect_object')
        service_name = (
            self.get_parameter('service_name')
            .get_parameter_value()
            .string_value
        )

        # Server del servizio
        self.__srv = self.create_service(
            DetectObject,
            service_name,
            self.__cb  # callback "privata" (name mangling) perché non pensata per override/uso esterno
        )

        self.get_logger().info(f"VisionSystem service ready on '{service_name}'")

    def __cb(self, request: DetectObject.Request, response: DetectObject.Response):
        self.get_logger().info(f"Received request for detecting object: {request.object_id}")

        r = self.__rng.random()  # [0.0, 1.0)

        if r < 0.5:
            self.get_logger().info(f"Object {request.object_id} detected!")

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'world'

            # Posizione casuale in [-5, 5]
            pose.pose.position.x = self.__rng.uniform(-5.0, 5.0)
            pose.pose.position.y = self.__rng.uniform(-5.0, 5.0)
            pose.pose.position.z = self.__rng.uniform(-5.0, 5.0)

            # Quaternione casuale 
            pose.pose.orientation.x = self.__rng.uniform(-1.0, 1.0)
            pose.pose.orientation.y = self.__rng.uniform(-1.0, 1.0)
            pose.pose.orientation.z = self.__rng.uniform(-1.0, 1.0)
            pose.pose.orientation.w = self.__rng.uniform(-1.0, 1.0)

            response.pose = pose
            response.success = True
        else:
            self.get_logger().info(f"Object {request.object_id} NOT detected!")
            response.success = False

        return response


def main(args=None):
    rclpy.init(args=args)
    node = VisionSystem()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
