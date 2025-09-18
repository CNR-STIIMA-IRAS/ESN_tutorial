import rclpy
from esn_tutorial_py.vision_server import VisionSystem

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
