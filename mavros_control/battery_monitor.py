import rclpy
from rclpy import Node

from sensor_msgs.msg import BatteryState

class BatteryMonitorNode(Node):
    def __init__(self):
        super.__init__()

        # TODO create subscriber to battery state node and create a timer that gets voltage every 30s

def main():
    node = BatteryMonitorNode()
    rclpy.spin_once(node)