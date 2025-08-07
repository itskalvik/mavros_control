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
        description = ParameterDescriptor(type =ParameterDescriptor.PARAMETER_DOUBLE, description="Voltage threshold for battery warning")
        self.declare_parameter('voltage_threshold', 14.9, description)  # Default threshold, can be changed via ROS parameter
        self.get_logger().info(f"Voltage threshold set to {self.get_parameter('voltage_threshold').get_parameter_value().double_value}")

        description = ParameterDescriptor(type =ParameterDescriptor.PARAMETER_DOUBLE, description="Current threshold for battery warning")
        self.declare_parameter('current_threshold', 0.8, description)  # Default current threshold, can be changed via ROS parameter
        self.get_logger().info(f"Current threshold set to {self.get_parameter('current_threshold').get_parameter_value().double_value}")

        self.volt_subscriber = self.create_subscription(BatteryState, '/mavros/battery', self.battery_callback, SENSOR_QOS)
        self.timer = self.create_timer(0.1,self.reading_callback)
    
    def battery_callback(self, msg: BatteryState):
        # get necessary information from battery topic
        self.volts = msg.voltage
        self.current = msg.current

    def reading_callback(self):

        # TODO: check voltage against threshold, add it to the buffer
        # TODO: in the future check discharge curve for different current draws

        # Debug line, replace later
        self.get_logger().info(f"calculating average voltage at 0.8 A readings: {self.volt_buffer}")

        voltage_threshold = self.get_parameter('voltage_threshold').get_parameter_value().double_value
        current_threshold = self.get_parameter('current_threshold').get_parameter_value().double_value

        if abs(abs(self.current) - current_threshold) < 0.1:
            self.volt_buffer.append(self.volts)
        if len(self.volt_buffer) > 5:
            self.volt_buffer.pop(0)
        if len(self.volt_buffer) >= 5:    
            if sum(self.volt_buffer)/5.0 <= voltage_threshold:
                self.get_logger().warn("VOLTAGE READING BELOW NOMINAL, CHARGE SOON")
            else:
                self.get_logger().info("Average voltage reading nominal")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()