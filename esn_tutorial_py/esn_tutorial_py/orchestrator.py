import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

from esn_msgs.srv import DetectObject
from esn_msgs.action import PickObject

import threading
import time


class Orchestrator(Node):
    def __init__(self):
        super().__init__("orchestrator_node")

        # Dichiarazione parametri
        self.declare_parameter("topic_name", "object_arrived")
        self.declare_parameter("service_name", "detect_object")
        self.declare_parameter("action_name", "take_object")
        self.declare_parameter("object_id", "target_1")

        self.limit_switch_topic = self.get_parameter("topic_name").value
        self.service_name = self.get_parameter("service_name").value
        self.action_name = self.get_parameter("action_name").value
        self.object_id = self.get_parameter("object_id").value

        # Stato interno
        self.busy = False
        self.object_arrived = False

        # Subscriber
        self.create_subscription(Bool, self.limit_switch_topic, self.on_limit_switch, 10)

        # Client servizio
        self.service_client = self.create_client(DetectObject, self.service_name)

        # Client azione
        self.action_client = ActionClient(self, PickObject, self.action_name)

        self.get_logger().info(
            f"Orchestrator ready. Subscribing '{self.limit_switch_topic}', "
            f"service '{self.service_name}', action '{self.action_name}'"
        )

        # Thread di esecuzione
        threading.Thread(target=self.execute, daemon=True).start()

        # self._lock = threading.Lock() In this case not necessary for GIL (Global Interpreter Lock). Take care if list, dict, ...

    # def set_busy(self, value: bool):
    #     with self._lock:
    #         self.busy = value
    # def get_busy(self) -> bool:
    #     with self._lock:
    #         return self.busy


    def on_limit_switch(self, msg: Bool):
        self.object_arrived = msg.data

    def go_pick_the_object(self):
        self.busy = True

        if not self.service_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f"Service '{self.service_name}' not available")
            self.busy = False
            return

        req = DetectObject.Request()
        req.object_id = self.object_id

        future = self.service_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        resp = future.result()
        if resp and resp.success:
            self.get_logger().info("VisionSystem: object detected")
            self.send_action_goal(resp.pose)
        else:
            self.get_logger().warn("VisionSystem: object NOT detected")

        self.busy = False

    def send_action_goal(self, pose: PoseStamped) -> bool:
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"Action server '{self.action_name}' not available")
            return False

        goal_msg = PickObject.Goal()
        goal_msg.pose = pose

        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.on_feedback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by action server")
            return False

        self.get_logger().info("Goal accepted")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        code = result_future.result().status

        if code == 4:  # STATUS_SUCCEEDED
            self.get_logger().info(f"Action SUCCEEDED, success={result.success}")
            return result.success
        elif code == 6:  # STATUS_ABORTED
            self.get_logger().warn("Action ABORTED")
        elif code == 2:  # STATUS_CANCELED
            self.get_logger().warn("Action CANCELED")
        else:
            self.get_logger().error("Unknown result code")
        return False

    def on_feedback(self, feedback_msg):
        self.get_logger().info(f"Progress: {feedback_msg.feedback.progress * 100.0:.0f}%")

    def execute(self):
        rate_hz = 10.0
        while rclpy.ok():
            if not self.busy and self.object_arrived:
                self.get_logger().info("-----------------------------")
                self.get_logger().info("New object arrived")
                self.go_pick_the_object()
            time.sleep(1.0 / rate_hz)

        self.get_logger().info("Stopping the orchestrator...")