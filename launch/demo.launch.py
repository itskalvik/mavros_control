from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.actions import PushRosNamespace
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction, IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    namespace = 'robot_0'
    nodes = []

    path_follower = Node(package='mavros_control',
                         executable='waypoint_path_follower',
                         namespace=namespace,
                         name='WaypointPathFollower',
                         parameters=[
                              {'xy_tolerance': 0.7,
                               'z_tolerance': 0.3,
                               'use_altitude': False,
                              }
                         ])
    nodes.append(path_follower)

    mavros = GroupAction(
                    actions=[
                        # push_ros_namespace to set namespace of included nodes
                        PushRosNamespace(namespace),
                        # MAVROS
                        IncludeLaunchDescription(
                            XMLLaunchDescriptionSource([
                                PathJoinSubstitution([
                                    FindPackageShare('ros_sgp_tools'),
                                    'launch',
                                    'mavros.launch'
                                ])
                            ]),
                            launch_arguments={
                            }.items()
                        ),
                    ]
                )
    nodes.append(mavros) 

    return LaunchDescription(nodes)