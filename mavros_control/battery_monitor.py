import rclpy
from rclpy.node import Node

from sensor_msgs.msg import BatteryState

class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__('battery_monitor_node')
        self.volts = 0.0
        self.volt_buffer = []
        self.current = 0.0

        SENSOR_QOS = rclpy.qos.qos_profile_sensor_data

        # TODO make voltage threshold a ROS parameter

        self.volt_subscriber = self.create_subscription(BatteryState, '/mavros/battery', self.battery_callback, SENSOR_QOS)
        self.timer = self.create_timer(30,self.reading_callback)
    
    def battery_callback(self, msg: BatteryState):
        # get necessary information from battery topic
        self.volts = msg.voltage
        self.current = msg.current

    def reading_callback(self):

        # TODO: check voltage against threshold, add it to the buffer
        # TODO: in the future check discharge curve for different current draws

        # Debug line, replace later
        self.get_logger().info(f"calculating average voltage... readings: {self.volt_buffer}")
        
        self.volt_buffer.append(self.volts)
        if len(self.volt_buffer) > 5:
            self.volt_buffer.pop(0)
        if len(self.volt_buffer) >= 5:    
            if sum(self.volt_buffer)/5.0 < 12.5:
                self.get_logger().warn("VOLTAGE READING BELOW 12.5, CHARGE SOON")
            else:
                self.get_logger().info("Average voltage reading nominal")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()