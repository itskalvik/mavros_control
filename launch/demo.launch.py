import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.actions import PushRosNamespace
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction, IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def get_var(var, default):
    try:
        return os.environ[var]
    except:
        return default

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
                               'navigation_type': 0,
                              }
                         ])
    nodes.append(path_follower)

    # MAVROS
    mavros = GroupAction(
                    actions=[
                        # push_ros_namespace to set namespace of included nodes
                        PushRosNamespace(namespace),
                        # MAVROS
                        IncludeLaunchDescription(
                            XMLLaunchDescriptionSource([
                                PathJoinSubstitution([
                                    FindPackageShare('mavros_control'),
                                    'launch',
                                    'mavros.launch'
                                ])
                            ]),
                            launch_arguments={
                                "fcu_url": "udp://0.0.0.0:14550@"
                            }.items()
                        ),
                    ]
                )
    nodes.append(mavros) 

    # Foxglove (web-based rviz)
    foxglove = GroupAction(
                    actions=[
                        # push_ros_namespace to set namespace of included nodes
                        PushRosNamespace(namespace),
                        # Foxglove
                        IncludeLaunchDescription(
                            XMLLaunchDescriptionSource([
                                PathJoinSubstitution([
                                    FindPackageShare('foxglove_bridge'),
                                    'launch',
                                    'foxglove_bridge_launch.xml'
                                ])
                            ]),
                        )
                    ]
                )
    nodes.append(foxglove)

    return LaunchDescription(nodes)