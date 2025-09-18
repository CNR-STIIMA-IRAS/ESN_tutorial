# dual_sub_node.py
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
import argparse

class DualSubNode(Node):
    def __init__(self):
        super().__init__('dual_sub_node')

        # Gruppo re-entrant: permette esecuzione concorrente della stessa callback
        # o di callback diverse sullo stesso nodo quando usi il MultiThreadedExecutor
        cg = ReentrantCallbackGroup()

        self.slow_sub = self.create_subscription(
            String, 'slow', self.slow_cb, 10, callback_group=cg
        )
        self.fast_sub = self.create_subscription(
            String, 'fast', self.fast_cb, 10, callback_group=cg
        )

    def slow_cb(self, msg: String):
        self.get_logger().info(f'[SLOW] start -> {msg.data}')
        # Simula lavoro bloccante/lento
        time.sleep(10.0)
        self.get_logger().info(f'[SLOW] done  -> {msg.data}')

    def fast_cb(self, msg: String):
        # Questa dovrebbe rimanere reattiva anche quando la slow è occupata
        self.get_logger().info(f'[FAST] {msg.data}')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--multi', action='store_true',
                        help='Usa MultiThreadedExecutor (2 thread)')
    args = parser.parse_args()

    rclpy.init()

    node = DualSubNode()

    if args.multi:
        execu = MultiThreadedExecutor(num_threads=2)
    else:
        execu = SingleThreadedExecutor()

    execu.add_node(node)
    try:
        execu.spin()
    finally:
        execu.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
