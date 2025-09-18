import rclpy
from esn_tutorial_py.orchestrator import Orchestrator


def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("User interrupted with Ctrl+C")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()
