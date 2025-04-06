"""
© 2025 Falk-Richard Kallmer, Leuphana University Lüneburg
MIT License – see LICENSE file for details
Contact: falk-richard.kallmer@stud.leuphana.de

Launches the main system for integrating MG400 and SLLiDAR (ROS 2 Humble) and visualization in RViz.
This script includes configurations for the MG400 robot, the SLLiDAR LiDAR, ultrasonic sensors, and MQTT communication.
"""

# Import necessary modules and libraries
import os
from pathlib import Path
from typing import Dict, List, Tuple

from ament_index_python.packages import get_package_share_path, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration,
    PathJoinSubstitution, TextSubstitution
)
from launch_ros.actions import Node

# Helper function to load the robot description from a Xacro file
# Loads the robot description from a Xacro or URDF file and returns it as a dictionary
def _load_robot_description(
    xacro_filepath: Path, xacro_options: List[Tuple] = None
) -> Dict:
    if 'xacro' in str(xacro_filepath):
        # Process Xacro file
        params = []
        if xacro_options:
            for option in xacro_options:
                params.append(' {}:='.format(option[0]))
                params.append(option[1])
        command = [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            str(xacro_filepath)
        ]
        robot_description_content = Command(command=(command + params))
    else:
        # Fallback: Read file directly
        try:
            with open(str(xacro_filepath), 'r') as file:
                robot_description_content = file.read()
        except EnvironmentError:
            exit(1)

    return {'robot_description': robot_description_content}

# Main function to generate the launch description
# Creates a launch description that includes all necessary nodes and configurations for integrating the MG400 robot and
# the SLLiDAR LiDAR.
def generate_launch_description():
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value=TextSubstitution(text='mg400'),
        description='Namespace for the robot resources.'
    )
    ip_address_arg = DeclareLaunchArgument(
        'ip_address',
        default_value=TextSubstitution(text='192.168.1.6'),
        description='IP address for connecting to the MG400.'
    )
    workspace_visible_arg = DeclareLaunchArgument(
        'workspace_visible',
        default_value=TextSubstitution(text='False'),
        description='Set to "True" to make the MG400 workspace visible in RViz.'
    )

    # MG400 robot configurations
    ns = LaunchConfiguration('namespace')
    ip_address = LaunchConfiguration('ip_address')
    workspace_visible = LaunchConfiguration('workspace_visible', default='False')

    # Declare launch arguments for the LiDAR
    channel_type_arg = DeclareLaunchArgument(
        'channel_type',
        default_value='serial',
        description='Channel type for the LiDAR (e.g., "serial").'
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for connecting the LiDAR.'
    )
    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='Baud rate for the LiDAR serial connection.'
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser',
        description='Frame ID for the LiDAR.'
    )
    inverted_arg = DeclareLaunchArgument(
        'inverted',
        default_value='false',
        description='Inverts the LiDAR scan data if required.'
    )
    angle_compensate_arg = DeclareLaunchArgument(
        'angle_compensate',
        default_value='true',
        description='Enables angle compensation for the LiDAR scan data.'
    )
    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode',
        default_value='Sensitivity',
        description='Scan mode used by the LiDAR.'
    )

    # Define the MG400 node
    # Starts the MG400 node (robot interface) with the specified parameters
    mg400_node = Node(
        package='mg400_node',
        executable='mg400_node_exec',
        name='mg400_node',
        namespace=ns,
        parameters=[{'ip_address': ip_address}],
        on_exit=Shutdown()
    )

    # Load the robot description
    xacro_filepath = get_package_share_path('integration_package') / 'urdf' / 'mg400.urdf.xacro'
    robot_description = _load_robot_description(
        xacro_filepath=xacro_filepath,
        xacro_options=dict(workspace_visible=workspace_visible).items()
    )

    # Define the Robot State Publisher node
    # Publishes the robot state based on the loaded robot description
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=ns,
        output='log',
        parameters=[robot_description]
    )

    # LiDAR configurations
    channel_type = LaunchConfiguration('channel_type')
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')
    inverted = LaunchConfiguration('inverted')
    angle_compensate = LaunchConfiguration('angle_compensate')
    scan_mode = LaunchConfiguration('scan_mode')

    # Define the SLLiDAR node
    # Starts the SLLiDAR node with the specified parameters
    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': channel_type,
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': frame_id,
            'inverted': inverted,
            'angle_compensate': angle_compensate,
            'scan_mode': scan_mode
        }],
        output='screen'
    )

    # Define the static TF node for the LiDAR
    # Publishes a static transformation between the robot and the LiDAR to define the LiDAR's position relative to the robot in RViz
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='mg400_laser_tf',
        arguments=[
            '0.6', '-0.1', '0.05',
            '3.141592', '0.0', '0.0',
            'arm_frame_link_offset',
            LaunchConfiguration('frame_id')
        ]
    )

    # Define the RViz node
    # Starts RViz with a predefined configuration for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=[
            '-d',
            str(get_package_share_path('integration_package') / 'rviz' / 'combined_view.rviz'),
            '--ros-args',
            '--log-level',
            'error'
        ]
    )

    # Define the MQTT node
    # Starts the MQTT node for communication with external systems (ESP32 and ultrasonic sensors)
    mqtt_node = Node(
        package='mqtt_bridge',
        executable='mqtt_node',
        name='mqtt_node',
        output='log'
    )

    # Define static TF nodes for ultrasonic sensors
    # Publishes static transformations for the ultrasonic sensors to define their positions relative to the robot in RViz
    ultrasonic_tf_x = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_ultrasonic_x',
        arguments=['0.05', '0', '-0.05', '0', '0', '0', 'mg400_link5', 'ultrasonic_x']
    )

    ultrasonic_tf_y = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_ultrasonic_y',
        arguments=['0', '0.05', '-0.05', '0', '0', '0', 'mg400_link5', 'ultrasonic_y']
    )

    ultrasonic_tf_z = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_ultrasonic_z',
        arguments=['0', '0', '-0.1', '0', '0', '0', 'mg400_link5', 'ultrasonic_z']
    )

    # Assemble the launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(namespace_arg)
    ld.add_action(ip_address_arg)
    ld.add_action(workspace_visible_arg)
    ld.add_action(channel_type_arg)
    ld.add_action(serial_port_arg)
    ld.add_action(serial_baudrate_arg)
    ld.add_action(frame_id_arg)
    ld.add_action(inverted_arg)
    ld.add_action(angle_compensate_arg)
    ld.add_action(scan_mode_arg)

    # Add nodes
    ld.add_action(mg400_node)
    ld.add_action(mqtt_node)
    ld.add_action(rsp_node)
    ld.add_action(sllidar_node)
    ld.add_action(static_tf_node)
    ld.add_action(rviz_node)
    ld.add_action(ultrasonic_tf_x)
    ld.add_action(ultrasonic_tf_y)
    ld.add_action(ultrasonic_tf_z)

    return ld
