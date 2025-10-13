import random, math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class CarTelemetry(Node):
    def __init__(self):
        super().__init__('car_telemetry')
        self.declare_parameter('car_id', 'car_11')
        self.declare_parameter('loss_prob', 0.15)  # rádiós "loss"
        self.declare_parameter('rate', 10.0)       # Hz
        self.car_id = self.get_parameter('car_id').get_parameter_value().string_value
        self.loss_p = self.get_parameter('loss_prob').get_parameter_value().double_value
        self.rate   = self.get_parameter('rate').get_parameter_value().double_value
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.pub = self.create_publisher(String, f'/car/{self.car_id}/telemetry', best_effort)
        self.t = 0.0
        self.timer = self.create_timer(1.0/self.rate, self.tick)
        self.get_logger().info(f'CarTelemetry for {self.car_id} ready.')

    def tick(self):
        self.t += 1.0/self.rate
        speed = 120 + 20*math.sin(0.6*self.t)
        rpm   = 5000 + 800*math.sin(1.1*self.t)
        ect   = 92 + 3*math.sin(0.15*self.t)
        if random.random() < self.loss_p:
            return
        msg = String()
        msg.data = f'speed={speed:.1f}, rpm={rpm:.0f}, ect={ect:.1f}'
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = CarTelemetry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
