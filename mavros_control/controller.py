#! /usr/bin/env python3

from mavros_msgs.srv import CommandHome, CommandTOL
from tf_transformations import euler_from_quaternion
from rcl_interfaces.msg import ParameterDescriptor
from mavros_msgs.msg import State, OverrideRCIn
from geographic_msgs.msg import GeoPoseStamped
from pygeodesy.geoids import GeoidPGM
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.node import Node
import rclpy

import numpy as np
from collections import deque
from .utils import *

_egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)


class Controller(Node):

    def __init__(self,
                 node_name='mavros_controller',
                 xy_tolerance=0.7,
                 z_tolerance=0.3,
                 use_altitude=False,
                 navigation_type=0,
                 start_mission=True):
        '''
        MAVROS waypoint path follower
        '''
        super().__init__(node_name)
        self.get_logger().info('Initializing')

        # Declare and get parameters
        description = ParameterDescriptor(description='XY-axis distance (meters) tolerance used to \
                                          determine if a waypoint is reached')
        self.declare_parameter('xy_tolerance', xy_tolerance, description)
        self.xy_tolerance = self.get_parameter('xy_tolerance').get_parameter_value().double_value
        self.get_logger().info(f'xy_tolerance: {self.xy_tolerance}')

        description = ParameterDescriptor(description='Z-axis distance (meters) tolerance used to \
                                          determine if a waypoint is reached')
        self.declare_parameter('z_tolerance', z_tolerance, description)
        self.z_tolerance = self.get_parameter('z_tolerance').get_parameter_value().double_value
        self.get_logger().info(f'z_tolerance: {self.z_tolerance}')

        description = ParameterDescriptor(description='If True 3D waypoints are used for the path')
        self.declare_parameter('use_altitude', use_altitude, description)
        self.use_altitude = self.get_parameter('use_altitude').get_parameter_value().bool_value
        self.get_logger().info(f'use_altitude: {self.use_altitude}')\
        
        description = ParameterDescriptor(description='\
                                          If 0 (default): Use global position based navigation (GPS-based).\n\
                                          If 1 Use raw rc controls for navigation.')
        self.declare_parameter('navigation_type', navigation_type, description)
        self.navigation_type = self.get_parameter('navigation_type').get_parameter_value().integer_value
        navigation_str = "GPS-based" if self.navigation_type==0 else "RC-control"
        self.get_logger().info(f'navigation_type: {navigation_str}')

        # Initialize variables
        self.vehicle_orientation = None
        self.vehicle_state = State()
        self.vehicle_position = np.array([0., 0., 0.])
        self.vehicle_linear = np.array([0., 0., 0.])
        self.heading_change = 0.01
        self.heading_buffer = deque([0.01])
        self.waypoint_distance = -1
        if self.navigation_type==0:
            self.setpoint_position = GeoPoseStamped()
        elif self.navigation_type==1:
            self.rc_override = OverrideRCIn()
            self.rc_override.channels = [0]*18

        # Create QoS profiles
        # STATE_QOS used for state topics, like ~/state, ~/mission/waypoints etc.
        STATE_QOS = rclpy.qos.QoSProfile(
            depth=10, 
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        # SENSOR_QOS used for most of sensor streams
        SENSOR_QOS = rclpy.qos.qos_profile_sensor_data

        # Create subscribers
        self.vehicle_state_subscriber = self.create_subscription(
            State, 'mavros/state', self.vehicle_state_callback, STATE_QOS)
        if self.navigation_type==0: # Global navigation
            self.vehicle_pose_subscriber = self.create_subscription(
                NavSatFix, 'mavros/global_position/global', 
                self.global_position_callback, SENSOR_QOS)
            self.vehicle_odom_subscriber = self.create_subscription(
                Odometry, 'mavros/local_position/odom',
                self.vehicle_odom_callback, SENSOR_QOS)
        elif self.navigation_type==1: # RC control
            self.vehicle_odom_subscriber = self.create_subscription(
                Imu, 'mavros/imu/data', 
                self.vehicle_odom_callback, SENSOR_QOS)
            
        # Create publishers
        if self.navigation_type==0: # Global navigation
            self.setpoint_position_publisher = self.create_publisher(
                GeoPoseStamped, 'mavros/setpoint_position/global', SENSOR_QOS)
        elif self.navigation_type==1: # RC control
            self.rc_override_publisher = self.create_publisher(
                OverrideRCIn, 'mavros/rc/override', STATE_QOS)
        
        # Create service clients
        self.arm_client = ArmingServiceClient()
        self.set_mode_client = SetModeServiceClient()

        # Wait to get the state of the vehicle
        rclpy.spin_once(self, timeout_sec=5.0)

        # Start mission
        if start_mission:
            if self.navigation_type==0:
                self.guided_mission()
            else:
                self.rc_control_mission()

    def haversine(self, pt1, pt2):
        """
        Calculate the great circle distance between two points
        on the earth's surface (specified in decimal degrees)
        https://stackoverflow.com/a/29546836

        Args:
            pt1 (ndarray, [n, 2]): Start location longitude and latitude 
            pt2 (ndarray, [n, 2]): End location longitude and latitude 
        """
        lon1, lat1 = pt1[:, 0], pt1[:, 1]
        lon2, lat2 = pt2[:, 0], pt2[:, 1]
        lon1, lat1, lon2, lat2 = map(np.radians, [lon1, lat1, lon2, lat2])
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = np.sin(dlat/2.0)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2.0)**2
        c = 2 * np.arcsin(np.sqrt(a))
        m = 6378.137 * c * 1000
        return m

    def at_waypoint(self, waypoint):
        """Check if the vehicle is at the waypoint."""
        self.waypoint_distance = self.haversine(self.vehicle_position[:2].reshape(1, -1), 
                                                np.array(waypoint[:2]).reshape(1, -1))[0]
        if self.use_altitude:
            z_dist = np.abs(self.vehicle_position[2] - waypoint[2])
        else: 
            z_dist = 0.0

        if self.waypoint_distance < self.xy_tolerance and z_dist < self.z_tolerance:
            return True
        else:
            return False
        
    def quaternion2rpy(self, orientation):
        orientation_list = [orientation.x, 
                            orientation.y, 
                            orientation.z, 
                            orientation.w]
        return euler_from_quaternion(orientation_list)
        
    def vehicle_odom_callback(self, msg):
        """Callback function for vehicle odom topic subscriber
            - Populates the vehicle orientation
            - Computes the nominal heading and linear
                For navigation_type==0 -> velocity
                For navigation_type==1 -> acceleration
        """
        if self.navigation_type==0:
            self.vehicle_orientation = self.quaternion2rpy(msg.pose.pose.orientation)
            self.vehicle_linear = [msg.twist.twist.linear.x,
                                   msg.twist.twist.linear.y, 
                                   msg.twist.twist.linear.z]
            heading_change = np.hypot(msg.twist.twist.linear.x,
                                      msg.twist.twist.linear.y)
        elif self.navigation_type==1:
            self.vehicle_orientation = self.quaternion2rpy(msg.orientation)
            self.vehicle_linear = [msg.linear_acceleration.x,
                                   msg.linear_acceleration.y,
                                   msg.linear_acceleration.z]
            heading_change = np.hypot(msg.linear_acceleration.x,
                                      msg.linear_acceleration.y)
                        
        # Maintain ring buffer to compute heading change
        if len(self.heading_buffer) > 50:
            self.heading_buffer.popleft()
        self.heading_buffer.append(heading_change)
        self.heading_change = np.mean(self.heading_buffer)

    def vehicle_state_callback(self, vehicle_state):
        """Callback function for vehicle_state topic subscriber."""
        self.vehicle_state = vehicle_state

    def global_position_callback(self, msg):
        """Callback function for vehicle position topic subscriber."""
        self.vehicle_position[0] = msg.latitude
        self.vehicle_position[1] = msg.longitude

        # Offset to convert ellipsoid to AMSL height
        offset = _egm96.height(self.vehicle_position[0], self.vehicle_position[1])
        self.vehicle_position[2] = msg.altitude-offset

    def arm(self, state=True, timeout_sec=30):
        """Arm/Disarm the vehicle"""
        return self.arm_client.arm(state, timeout_sec)
    
    def set_mode(self, mode=True, timeout_sec=30):
        """Set the vehicle mode
        Valid flight modes: MANUAL, STABILIZE, ALT_HOLD
        """
        return self.set_mode_client.set_mode(mode, timeout_sec)
    
    def tol_command(self, altitude=20.0, timeout=30, time_lim=900):
        """Take Off/Land the vehicle to the given height

        Args:
            altitude (float): If greater than the curent height, the vehicle will take off and land otherwise
            
        """
        if altitude > self.self.vehicle_position[2]:
            str_state = 'takeoff'
        else:
            str_state = 'land'

        client = self.create_client(CommandTOL, f'mavros/cmd/{str_state}')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{str_state} service not available, waiting again...')
        self.get_logger().info(f'{str_state} service available')

        takeoff_request = CommandTOL.Request()
        takeoff_request.min_pitch = 0.0
        takeoff_request.yaw = 0.0
        takeoff_request.latitude = self.vehicle_position[0]
        takeoff_request.longitude = self.vehicle_position[1]
        takeoff_request.altitude = altitude

        # Send request
        future = client.call_async(takeoff_request)
        self.get_logger().info(f'{str_state} command sent')
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result():
            waypoint = [self.vehicle_position[0], 
                        self.vehicle_position[1],
                        altitude]
            start_time = self.get_clock().now().to_msg().sec
            while not self.at_waypoint(waypoint):
                if self.get_clock().now().to_msg().sec - start_time > time_lim:
                    self.get_logger().info(f'Timeout: Failed to {str_state}')
                    return False
                rclpy.spin_once(self, timeout_sec=1.0)
            return True
        else: 
            self.get_logger().info(f'COMMAND_ACK: Failed to {str_state}')
            return False
    
    def set_home(self, latitude, longitude, altitude=None, timeout=900):
        """Set the home position"""

        home_client = self.create_client(CommandHome, 'mavros/cmd/set_home')
        while not home_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set home service not available, waiting again...')
        self.get_logger().info('Set home service available')

        set_home_request = CommandHome.Request()
        set_home_request.latitude = latitude
        set_home_request.longitude = longitude
        if altitude is not None:
            set_home_request.altitude = altitude

        future = home_client.call_async(set_home_request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.result().success:
            self.get_logger().info(f'Timeout: Failed to set home')
            return False

        return True
    
    def go2waypoint(self, waypoint, timeout=900):
        """
        Go to waypoint when in GUIDED mode and armed

        Args:
            waypoint (list): [latitude, longitude, altitude (optional)]
            timeout (double): Maximum amount of time to wait to reach the given waypoint
        """

        self.setpoint_position.pose.position.latitude = waypoint[0]
        self.setpoint_position.pose.position.longitude = waypoint[1]
        if self.use_altitude:
            self.setpoint_position.pose.position.altitude = waypoint[2]
        else:
            waypoint = waypoint[:2]

        start_time = self.get_clock().now().to_msg().sec
        last_request = start_time-6.0
        while not self.at_waypoint(waypoint):
            # Send the command only once every 5 seconds
            if self.get_clock().now().to_msg().sec - last_request < 5.0:
                rclpy.spin_once(self, timeout_sec=1.0)
                continue

            self.setpoint_position.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_position_publisher.publish(self.setpoint_position)
            last_request = self.get_clock().now().to_msg().sec

            # Timeout
            if not self.at_waypoint(waypoint) and \
                last_request - start_time > timeout:
               
                self.get_logger().info(f'Timeout: Failed to go to waypoint: {waypoint[0]}, {waypoint[1]}')
                return False

        return True

    def normalize(self, x, inmin=-1, inmax=1, outmin=1100, outmax=1900):
        # Function to map input range to output range
        return round((x - inmin) * (outmax - outmin) / (inmax - inmin) + outmin)
    
    def pub_rc_override(self, cmd, timeout=0.01):
        """
        Publishes rc messages, assumes MANUAL/STABILIZE/ALT_HOLD mode and armed

        Args:
            cmd (list): [Forward, Lateral, Throttle (Up/Down), Roll, Pitch, Yaw];
                        Input range: [-1.0, 1.0]
            timeout (double): Amount of time in seconds to publish the command message
        """

        # Forward
        self.rc_override.channels[4] = self.normalize(cmd[0])
        # Lateral
        self.rc_override.channels[5] = self.normalize(cmd[1])
        # Throttle (Up/Down)
        self.rc_override.channels[2] = self.normalize(cmd[2])
        # Roll
        self.rc_override.channels[1] = self.normalize(cmd[3])
        # Pitch
        self.rc_override.channels[0] = self.normalize(cmd[4])
        # Yaw
        self.rc_override.channels[3] = self.normalize(cmd[5])

        # Publish the message
        self.rc_override_publisher.publish(self.rc_override)
        start_time = self.get_clock().now().to_msg().sec
        while self.get_clock().now().to_msg().sec - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.01)
            self.rc_override_publisher.publish(self.rc_override)

        return True
        
    def guided_mission(self):
        """Valid flight modes: GUIDED"""
        mission_altitude = self.vehicle_position[2]

        self.get_logger().info('Engaging GUIDED mode')
        if self.set_mode('GUIDED'):
            self.get_logger().info('GUIDED mode Engaged')

        self.get_logger().info('Arming')
        if self.arm(True):
            self.get_logger().info('Armed')

        self.get_logger().info('Setting current positon as home')
        if self.set_home(self.vehicle_position[0], 
                         self.vehicle_position[1]):
            self.get_logger().info('Home position set')

        if self.use_altitude:
            if self.tol_command(mission_altitude):
                self.get_logger().info('Takeoff complete')

        self.get_logger().info('Moving 10m to the right')
        self.go2waypoint([self.vehicle_position[0], 
                          self.vehicle_position[1]+(10/111111), 
                          mission_altitude])
        
        self.get_logger().info('Moving 10m forward')
        self.go2waypoint([self.vehicle_position[0]+(10/111111), 
                          self.vehicle_position[1], 
                          mission_altitude])
        
        self.get_logger().info('Moving 10m to the left')
        self.go2waypoint([self.vehicle_position[0], 
                          self.vehicle_position[1]-(10/111111), 
                          mission_altitude])
        
        self.get_logger().info('Moving 10m backward')
        self.go2waypoint([self.vehicle_position[0]-(10/111111), 
                          self.vehicle_position[1], 
                          mission_altitude])

        if self.use_altitude:
            if self.tol_command(mission_altitude):
                self.get_logger().info('Landing complete')

        self.get_logger().info('Disarming')
        if self.arm(False):
            self.get_logger().info('Disarmed')

    def rc_control_mission(self):
        """Valid flight modes: MANUAL, STABILIZE, ALT_HOLD"""
        mission_altitude = self.vehicle_position[2]

        self.get_logger().info('Engaging MANUAL mode')
        # ArduSub internally used ALT_HOLD for DEPTH_HOLD
        if self.set_mode('MANUAL'):
            self.get_logger().info('MANUAL mode Engaged')

        self.get_logger().info('Arming')
        if self.arm(True):
            self.get_logger().info('Armed')

        if self.use_altitude:
            if self.tol_command(mission_altitude+20.0):
                self.get_logger().info('Takeoff complete')

        self.get_logger().info('Moving Down')
        self.pub_rc_override([0, 0, -1.0, 0, 0, 0], timeout=3)

        self.get_logger().info('Roll Right')
        self.pub_rc_override([0, 0, 0, 5.0, 0, 0], timeout=3)

        self.get_logger().info('Roll Left')
        self.pub_rc_override([0, 0, 0, -5.0, 0, 0], timeout=3)

        self.get_logger().info('Pitch Up')
        self.pub_rc_override([0, 0, 0, 0, 5.0, 0], timeout=3)

        self.get_logger().info('Pitch Down')
        self.pub_rc_override([0, 0, 0, 0, -5.0, 0], timeout=3)

        self.get_logger().info('Yaw Right')
        self.pub_rc_override([0, 0, 0, 0, 0, 0.25], timeout=3)

        self.get_logger().info('Yaw Left')
        self.pub_rc_override([0, 0, 0, 0, 0, -0.25], timeout=3)

        self.get_logger().info('Moving Forward')
        self.pub_rc_override([0.5, 0, 0, 0, 0, 0], timeout=3)

        self.get_logger().info('Moving Backward')
        self.pub_rc_override([-0.5, 0, 0, 0, 0, 0], timeout=3)

        self.get_logger().info('Moving Right')
        self.pub_rc_override([0.0, 0.5, 0, 0, 0, 0], timeout=3)
        
        self.get_logger().info('Moving Left')
        self.pub_rc_override([0, -0.5, 0, 0, 0, 0], timeout=3)

        self.get_logger().info('Moving Down')
        self.pub_rc_override([0, 0, -0.5, 0, 0, 0], timeout=3)

        self.get_logger().info('Moving Up')
        self.pub_rc_override([0, 0, 0.5, 0, 0, 0], timeout=3)

        if self.use_altitude:
            if self.tol_command(mission_altitude):
                self.get_logger().info('Landing complete')

        self.get_logger().info('Disarming')
        if self.arm(False):
            self.get_logger().info('Disarmed')


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin_once(node)

if __name__ == '__main__':
    main()
