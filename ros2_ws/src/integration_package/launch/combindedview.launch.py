"""Launch the main system integrating MG400 and SLLiDAR (ROS 2 Humble)."""

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


def _load_robot_description(
    xacro_filepath: Path, xacro_options: List[Tuple] = None
) -> Dict:
    """
    Load the robot description from a file.

    If the file is a xacro file, process it with the given options.
    Otherwise, simply read the file contents.
    """
    if 'xacro' in str(xacro_filepath):
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
        try:
            with open(str(xacro_filepath), 'r') as file:
                robot_description_content = file.read()
        except EnvironmentError:
            exit(1)

    return {'robot_description': robot_description_content}


def generate_launch_description():
    """Generate and return the LaunchDescription for MG400 and optional SLLiDAR integration."""

    # ---------------------------------------------------
    #            MG400-Specific Launch Arguments
    # ---------------------------------------------------
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

    ns = LaunchConfiguration('namespace')
    ip_address = LaunchConfiguration('ip_address')
    workspace_visible = LaunchConfiguration('workspace_visible', default='False')

    # ---------------------------------------------------
    #            SLLiDAR-Specific Launch Arguments
    # ---------------------------------------------------
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
        description='Frame ID for the LiDAR data.'
    )
    inverted_arg = DeclareLaunchArgument(
        'inverted',
        default_value='false',
        description='Invert the LiDAR scan data if needed.'
    )
    angle_compensate_arg = DeclareLaunchArgument(
        'angle_compensate',
        default_value='true',
        description='Enable angle compensation for the LiDAR scan data.'
    )
    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode',
        default_value='Sensitivity',
        description='Scan mode to be used by the LiDAR.'
    )

    # ---------------------------------------------------
    #            MG400 Control Node
    # ---------------------------------------------------
    mg400_node = Node(
        package='mg400_node',
        executable='mg400_node_exec',
        name='mg400_node',
        namespace=ns,
        parameters=[{'ip_address': ip_address}],
        on_exit=Shutdown()
    )

    # Load the robot description (URDF) via xacro processing
    xacro_filepath = get_package_share_path('integration_package') / 'urdf' / 'mg400.urdf.xacro'
    robot_description = _load_robot_description(
        xacro_filepath=xacro_filepath,
        xacro_options=dict(workspace_visible=workspace_visible).items()
    )

    # Robot State Publisher for broadcasting transforms
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=ns,
        output='log',
        parameters=[robot_description]
    )

    # ---------------------------------------------------
    #            SLLiDAR Node
    # ---------------------------------------------------
    channel_type = LaunchConfiguration('channel_type')
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')
    inverted = LaunchConfiguration('inverted')
    angle_compensate = LaunchConfiguration('angle_compensate')
    scan_mode = LaunchConfiguration('scan_mode')

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

    # ---------------------------------------------------
    #            Static TF Transformation
    # ---------------------------------------------------
    # Define a fixed transformation between the robot base frame and the LiDAR frame.
    # Adjust the translation (x, y, z in meters) and rotation (roll, pitch, yaw in radians)
    # to match your actual hardware mounting.
    # Note: The base frame may be named differently in your URDF (e.g., "mg400_base_link" or "arm_frame_link_offset").
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='mg400_laser_tf',
        arguments=[
            # Translation: x, y, z (meters)
            '0.6', '-0.1', '0.05',
            # Rotation: roll, pitch, yaw (radians)
            '3.141592', '0.0', '0.0',
            # Parent frame: robot base frame
            'arm_frame_link_offset',
            # Child frame: LiDAR frame (default is "laser")
            LaunchConfiguration('frame_id')
        ]
    )

    # ---------------------------------------------------
    #            RViz Visualization
    # ---------------------------------------------------
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

    # ---------------------------------------------------
    #            MQTT Bridge Node
    # ---------------------------------------------------
    mqtt_node = Node(
        package='mqtt_bridge',
        executable='mqtt_node',
        name='mqtt_node',
        output='log'
    )

    # ---------------------------------------------------
    #            Static TF Transformation Ultrasonic
    # ---------------------------------------------------

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
        arguments=['0', '-0.05', '-0.05', '-1.5707963', '0.0', '0.0', 'mg400_link5', 'ultrasonic_y']
    )

    ultrasonic_tf_z = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_ultrasonic_z',
        arguments=['0', '0.05', '-0.05', '1.5707963', '0.0', '0.0', 'mg400_link5', 'ultrasonic_z']
    )

    # ---------------------------------------------------
    #            Assemble LaunchDescription
    # ---------------------------------------------------
    ld = LaunchDescription()

    # Add MG400-specific launch arguments
    ld.add_action(namespace_arg)
    ld.add_action(ip_address_arg)
    ld.add_action(workspace_visible_arg)

    # Add SLLiDAR-specific launch arguments
    ld.add_action(channel_type_arg)
    ld.add_action(serial_port_arg)
    ld.add_action(serial_baudrate_arg)
    ld.add_action(frame_id_arg)
    ld.add_action(inverted_arg)
    ld.add_action(angle_compensate_arg)
    ld.add_action(scan_mode_arg)

    # Add nodes to the launch description
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