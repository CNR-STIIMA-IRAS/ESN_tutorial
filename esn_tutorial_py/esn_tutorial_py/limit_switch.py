import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.duration import Duration


class LimitSwitch(Node):
    def __init__(self):
        super().__init__('limit_switch_node')

        # Dichiarazione e lettura dei parametri
        self.declare_parameter('topic_name', '/object_arrived')
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        self.declare_parameter('delta_time', 10.0)
        self.delta_t = self.get_parameter('delta_time').get_parameter_value().double_value

        # Publisher
        self.publisher = self.create_publisher(Bool, topic_name, 10)

        # Tempo dell'ultima pubblicazione "True"
        self.last_publish_time = self.get_clock().now()

        # Timer richiamato ogni 1 secondo
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info(
            f"LimitSwitch node started. Publishing on '{topic_name}' every 1s. "
            f"True every {self.delta_t} seconds."
        )

    def timer_callback(self):
        now = self.get_clock().now()
        msg = Bool()

        if (now - self.last_publish_time) > Duration(seconds=self.delta_t):
            msg.data = True
            self.last_publish_time = now
        else:
            msg.data = False

        self.publisher.publish(msg)
        self.get_logger().debug(f"Published: {msg.data}")


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
