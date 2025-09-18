import rclpy
from esn_tutorial_py.limit_switch import LimitSwitch


def main(args=None):
    rclpy.init(args=args)
    node = LimitSwitch()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
