import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class RaceControl(Node):
    def __init__(self):
        super().__init__('race_control')
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.control_pub = self.create_publisher(String, '/race/control', reliable_qos)
        self.alert_pub   = self.create_publisher(String, '/alerts', reliable_qos)
        self.srv = self.create_service(Trigger, '/request_stage_start', self.handle_stage_start)
        self.get_logger().info('RaceControl ready.')
        self.timer = self.create_timer(5.0, self.send_heartbeat)

    def send_heartbeat(self):
        msg = String()
        msg.data = 'CTRL HEARTBEAT'
        self.control_pub.publish(msg)

    def handle_stage_start(self, request, response):
        m = String()
        m.data = 'SS1 START'
        self.control_pub.publish(m)
        response.success = True
        response.message = 'Stage start command broadcasted.'
        return response

def main():
    rclpy.init()
    node = RaceControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
