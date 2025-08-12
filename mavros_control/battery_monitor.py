import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from sensor_msgs.msg import BatteryState

class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__('battery_monitor_node')
        self.volts = 0.0
        self.volt_buffer = []
        self.current = 0.0

        SENSOR_QOS = rclpy.qos.qos_profile_sensor_data


        # Declaring threshold parameters with default values
        description = ParameterDescriptor(description="Voltage threshold for battery warning")
        self.declare_parameter('voltage_threshold', 14.8, description)  # Default threshold, can be changed via ROS parameter
        self.get_logger().info(f"Voltage threshold set to {self.get_parameter('voltage_threshold').get_parameter_value().double_value}")

        description = ParameterDescriptor(description="Current threshold for battery warning")
        self.declare_parameter('current_threshold', 0.8, description)  # Default current threshold, can be changed via ROS parameter
        self.get_logger().info(f"Current threshold set to {self.get_parameter('current_threshold').get_parameter_value().double_value}")

        self.volt_subscriber = self.create_subscription(BatteryState, '/mavros/battery', self.battery_callback, SENSOR_QOS)
        self.timer = self.create_timer(30, self.reading_callback)
    
    def battery_callback(self, msg: BatteryState):
        # get necessary information from battery topic
        self.volts = msg.voltage
        self.current = msg.current

    def reading_callback(self):

        # TODO: in the future check discharge curve for different current draws
        # TODO: use discharge graph to take in voltage and current and get what the voltage would be at 0A

        voltage_threshold = (-0.0285*(abs(self.current)) + 3.73) * 4

        if self.volts < voltage_threshold:
            self.get_logger().warn(f"Voltage below threshold at {self.current} A: {self.volts} V < {voltage_threshold} V")
        else:
            self.get_logger().info(f"Voltage nominal: {self.volts}V at {abs(self.current)}A with voltage threshold {voltage_threshold} V")

        

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

