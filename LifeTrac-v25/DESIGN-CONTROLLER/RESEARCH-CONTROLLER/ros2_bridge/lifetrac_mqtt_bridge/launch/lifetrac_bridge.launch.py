#!/usr/bin/env python3
"""
Launch file for LifeTrac v25 ROS2-MQTT Bridge

This launch file starts both the mqtt_client node and the lifetrac_mqtt_bridge node
to enable ROS2 control of the LifeTrac v25 system from a BeagleBone or any ROS2-enabled device.

Usage:
    ros2 launch lifetrac_mqtt_bridge lifetrac_bridge.launch.py

Optional arguments:
    broker_host:=<ip>       MQTT broker IP address (default: 192.168.1.100)
    broker_port:=<port>     MQTT broker port (default: 1883)
    mqtt_user:=<username>   MQTT username (default: lifetrac)
    mqtt_pass:=<password>   MQTT password (default: lifetrac_pass)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('lifetrac_mqtt_bridge')
    
    # Declare launch arguments
    broker_host_arg = DeclareLaunchArgument(
        'broker_host',
        default_value='192.168.1.100',
        description='MQTT broker host IP address'
    )
    
    broker_port_arg = DeclareLaunchArgument(
        'broker_port',
        default_value='1883',
        description='MQTT broker port'
    )
    
    mqtt_user_arg = DeclareLaunchArgument(
        'mqtt_user',
        default_value='lifetrac',
        description='MQTT broker username'
    )
    
    mqtt_pass_arg = DeclareLaunchArgument(
        'mqtt_pass',
        default_value='lifetrac_pass',
        description='MQTT broker password'
    )
    
    # Parameters file path
    params_file = os.path.join(pkg_dir, 'config', 'mqtt_bridge_params.yaml')
    
    # MQTT Client node (from ika-rwth-aachen/mqtt_client)
    # This node handles the actual MQTT communication
    mqtt_client_node = Node(
        package='mqtt_client',
        executable='mqtt_client',
        name='mqtt_client',
        output='screen',
        parameters=[
            params_file,
            {
                'broker.host': LaunchConfiguration('broker_host'),
                'broker.port': LaunchConfiguration('broker_port'),
                'broker.user': LaunchConfiguration('mqtt_user'),
                'broker.pass': LaunchConfiguration('mqtt_pass'),
            }
        ],
        remappings=[
            # Map mqtt_client topics if needed
        ]
    )
    
    # LifeTrac MQTT Bridge node
    # This node provides additional logging and validation
    lifetrac_bridge_node = Node(
        package='lifetrac_mqtt_bridge',
        executable='mqtt_bridge_node',
        name='lifetrac_mqtt_bridge',
        output='screen',
        parameters=[
            params_file,
            {
                'mqtt_broker_host': LaunchConfiguration('broker_host'),
                'mqtt_broker_port': LaunchConfiguration('broker_port'),
                'mqtt_username': LaunchConfiguration('mqtt_user'),
                'mqtt_password': LaunchConfiguration('mqtt_pass'),
            }
        ]
    )
    
    # Launch info message
    launch_info = LogInfo(
        msg=[
            'Starting LifeTrac v25 ROS2-MQTT Bridge\n',
            'MQTT Broker: ', LaunchConfiguration('broker_host'), ':', 
            LaunchConfiguration('broker_port'), '\n',
            'Control commands can be sent to: /lifetrac/control_cmd\n',
            'Status messages will be available on: /lifetrac/status'
        ]
    )
    
    return LaunchDescription([
        broker_host_arg,
        broker_port_arg,
        mqtt_user_arg,
        mqtt_pass_arg,
        launch_info,
        mqtt_client_node,
        lifetrac_bridge_node,
    ])
