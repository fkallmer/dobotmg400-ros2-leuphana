# bringup.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mqtt_bridge',           # Paket deiner MQTT-Bridge
            executable='mqtt_bridge',         # Name der MQTT-Bridge-Executable
            name='mqtt_bridge'
        ),
        Node(
            package='ultrasonic_control',            # Unser neues Paket
            executable='ultrasonic_control_node',            # Das Executable, siehe CMakeLists.txt
            name='ultrasonic_control_node'
        ),
    ])
