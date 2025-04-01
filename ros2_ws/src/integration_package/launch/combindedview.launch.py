"""Launch main system with MG400 + SLLiDAR integration (ROS 2 Humble)."""
# Copyright 2022 HarvestX Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from pathlib import Path
from typing import Dict, List, Tuple

from ament_index_python.packages import get_package_share_path, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution, TextSubstitution)
from launch_ros.actions import Node


def _load_robot_description(
    xacro_filepath: Path, xacro_options: List[Tuple] = False
) -> Dict:
    """Load robot description."""
    if 'xacro' in str(xacro_filepath):
        params = []
        if xacro_options:
            for xacro_option in xacro_options:
                params.append(' {}:='.format(xacro_option[0]))
                params.append(xacro_option[1])
        command = [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            str(xacro_filepath)
        ]
        robot_description_content = Command(command=(command + params))
    else:
        try:
            with open(str(xacro_filepath), 'r') as file:
                robot_description_content = file
        except EnvironmentError:
            exit(1)

    return {'robot_description': robot_description_content}


def generate_launch_description():
    """Launch MG400 system + optional SLLiDAR (ROS 2 Humble)."""

    # ---------------------------------------------------
    #            MG400-spezifische Argumente
    # ---------------------------------------------------
    ns_arg = DeclareLaunchArgument(
        'namespace',
        default_value=TextSubstitution(text='mg400'),
        description='Set the robot resource namespace.'
    )
    ns = LaunchConfiguration('namespace')

    ip_address_arg = DeclareLaunchArgument(
        'ip_address',
        default_value=TextSubstitution(text='192.168.1.6'),
        description='Set the ip address to connect'
    )
    ip_address = LaunchConfiguration('ip_address')

    workspace_visible_arg = DeclareLaunchArgument(
        'workspace_visible',
        default_value=TextSubstitution(text='False'),
        description='true : MG400 workspace is visible in rviz'
    )
    workspace_visible = LaunchConfiguration('workspace_visible', default='False')

    # ---------------------------------------------------
    #            SLLiDAR-spezifische Argumente
    # ---------------------------------------------------
    channel_type_arg = DeclareLaunchArgument(
        'channel_type',
        default_value='serial',
        description='Specifying channel type of lidar'
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Specifying usb port to connected lidar'
    )
    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='Specifying usb port baudrate to connected lidar'
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser',
        description='Specifying frame_id of lidar'
    )
    inverted_arg = DeclareLaunchArgument(
        'inverted',
        default_value='false',
        description='Specifying whether or not to invert scan data'
    )
    angle_compensate_arg = DeclareLaunchArgument(
        'angle_compensate',
        default_value='true',
        description='Specifying whether or not to enable angle_compensate of scan data'
    )
    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode',
        default_value='Sensitivity',
        description='Specifying scan mode of lidar'
    )

    # ---------------------------------------------------
    #            MG400 Nodes
    # ---------------------------------------------------
    mg400_node = Node(
        package='mg400_node',
        executable='mg400_node_exec',
        name='mg400_node',
        namespace=ns,
        parameters=[{
            'ip_address': ip_address,
        }],
        on_exit=Shutdown()
    )

    xacro_filepath_ = get_package_share_path('integration_package') / 'urdf' / 'mg400.urdf.xacro'
    robot_description = _load_robot_description(
        xacro_filepath=xacro_filepath_,
        xacro_options={
            'workspace_visible': workspace_visible,
        }.items()
    )

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
    #            Statische TF-Transformation
    # ---------------------------------------------------
    # Hier ein Beispiel: Falls der Laser starr auf dem mg400 montiert ist,
    # kannst du eine feste Transformation von mg400_base_link -> laser definieren.
    # Passen ggf. x, y, z, roll, pitch, yaw an die reale Montage an.
    #
    # ACHTUNG: Dein URDF könnte statt "mg400_base_link" auch "arm_frame_link_offset"
    # als Basis-Frame haben. Bitte sicherstellen, welches Frame wirklich genutzt wird.
    #
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='mg400_laser_tf',
        arguments=[
            # x, y, z (in Metern)
            '0.6', '-0.1', '0.05',
            # roll, pitch, yaw (in Radianten)
            '3.15', '0.0', '0.0',
            # Parent frame (MG400-Basis)
            'arm_frame_link_offset',
            # Child frame (Lidar)
            LaunchConfiguration('frame_id'),  # = "laser" per default
        ]
    )

    # ---------------------------------------------------
    #            RViz
    # ---------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=[
            '-d',
            str(
                get_package_share_path('integration_package')
                / 'rviz'
                / 'combined_view.rviz'
            ),
            '--ros-args',
            '--log-level',
            'error'
        ]
    )

    # ---------------------------------------------------
    #            LaunchDescription zusammensetzen
    # ---------------------------------------------------
    ld = LaunchDescription()

    # MG400-Argumente
    ld.add_action(ns_arg)
    ld.add_action(ip_address_arg)
    ld.add_action(workspace_visible_arg)

    # SLLiDAR-Argumente
    ld.add_action(channel_type_arg)
    ld.add_action(serial_port_arg)
    ld.add_action(serial_baudrate_arg)
    ld.add_action(frame_id_arg)
    ld.add_action(inverted_arg)
    ld.add_action(angle_compensate_arg)
    ld.add_action(scan_mode_arg)

    # Nodes
    ld.add_action(mg400_node)
    ld.add_action(rsp_node)
    ld.add_action(sllidar_node)
    ld.add_action(static_tf_node)  # <--- statische TF wird hier eingebunden

    # Aktuelles MG400-RViz starten
    ld.add_action(rviz_node)

    return ld
