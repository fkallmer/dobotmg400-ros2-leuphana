"""
© 2025 Falk-Richard Kallmer, Leuphana University Lüneburg
MIT License – see LICENSE file for details
Contact: falk-richard.kallmer@stud.leuphana.de

This script initializes the ultrasonic control system for the MG400 robot and the neede MQTT-Bridge.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Create and return a LaunchDescription object that defines the nodes to be launched
    return LaunchDescription([
        # Launch the MQTT bridge node
        Node(
            package='mqtt_bridge',           # The package containing the MQTT bridge
            executable='mqtt_bridge',        # The executable for the MQTT bridge
            name='mqtt_bridge'               # The name of the node in the ROS2 system
        ),
        # Launch the ultrasonic control node
        Node(
            package='ultrasonic_control',    # The package containing the ultrasonic control node
            executable='ultrasonic_control_node',  # The executable for the ultrasonic control node
            name='ultrasonic_control_node'  # The name of the node in the ROS2 system
        ),
    ])
