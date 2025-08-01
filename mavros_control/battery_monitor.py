import rclpy
from rclpy import Node

from sensor_msgs.msg import BatteryState

class BatteryMonitorNode(Node):
    def __init__(self):
        super.__init__()
        self.volts = 0.0
        self.volt_buffer = []
        self.current = 0.0

        self.volt_subscriber = self.create_subscription(BatteryState, '/mavros/battery', self.battery_callback, 10)
        self.timer = self.create_timer(30,self.reading_callback)
    
    def battery_callback(self, msg: BatteryState):
        
        # TODO: get necessary information from battery topic
        self.volts = msg.voltage
        self.current = msg.current
        pass

    def reading_callback(self):

        # TODO: check voltage against threshold, add it to the buffer
        self.volt_buffer.append(self.volts)
        if len(self.volt_buffer) > 5:
            self.volt_buffer.pop(0)
        if sum(self.volt_buffer)/5.0 < 12.0:
            self.get_logger().warn("VOLTAGE READING BELOW 12, CHARGE SOON")
        pass
    
def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    rclpy.spin_once(node)

if __name__ == '__main__':
    main()