#!/usr/bin/env python3

import rclpy
from esn_tutorial_py.robot_action_server import RobotActionServer


def main(args=None):
    rclpy.init(args=args)
    node = RobotActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
