import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class StageMarshal(Node):
    def __init__(self):
        super().__init__('stage_marshal')
        self.declare_parameter('stage', 'SS1')
        self.stage = self.get_parameter('stage').get_parameter_value().string_value
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.events_pub = self.create_publisher(String, f'/marshal/{self.stage.lower()}/events', reliable_qos)
        self.control_sub = self.create_subscription(String, '/race/control', self.on_control, reliable_qos)
        self.cli = self.create_client(Trigger, '/request_stage_start')
        self.get_logger().info(f'StageMarshal for {self.stage} ready.')
        self.create_timer(3.0, self.request_start_once)
        self._requested = False

    def request_start_once(self):
        if self._requested or not self.cli.service_is_ready():
            return
        req = Trigger.Request()
        self._requested = True
        future = self.cli.call_async(req)
        future.add_done_callback(self.after_request)

    def after_request(self, future):
        try:
            resp = future.result()
            self.get_logger().info(f'Start response: {resp.success} {resp.message}')
        except Exception as e:
            self.get_logger().error(f'Start request failed: {e}')

    def on_control(self, msg: String):
        if msg.data.startswith(f'{self.stage} START'):
            if random.random() < 0.3:
                em = String()
                em.data = f'{self.stage} OBSTACLE @KM{round(random.uniform(0.5, 4.5), 1)}'
                self.events_pub.publish(em)
                self.get_logger().warn(em.data)

def main():
    rclpy.init()
    node = StageMarshal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
