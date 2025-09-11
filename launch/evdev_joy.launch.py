#!/usr/bin/env python3
"""Launch file für den evdev_joy_node des Pakets wsl_joystick_bridge.

Beispiel:
  ros2 launch wsl_joystick_bridge evdev_joy.launch.py device_path:=/dev/input/event3 joy_topic:=/gamepad deadzone:=0.12 frame_id:=joy_link
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch Arguments deklarieren
    joy_topic_arg = DeclareLaunchArgument(
        'joy_topic', default_value='joy',
        description='Topic Name für sensor_msgs/Joy Ausgabe')
    device_path_arg = DeclareLaunchArgument(
        'device_path', default_value='/dev/input/event0',
        description='Pfad zum evdev Input Device (z.B. /dev/input/event3)')
    deadzone_arg = DeclareLaunchArgument(
        'deadzone', default_value='0.1',
        description='Deadzone (0.0 - <1.0) für Analogachsen')
    frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='',
        description='Optionales frame_id im Header der Joy Nachricht')
    reconnect_seconds_arg = DeclareLaunchArgument(
        'reconnect_seconds', default_value='3.0',
        description='Sekunden zwischen Reconnect-Versuchen falls Device fehlt')

    # Node Definition
    joy_node = Node(
        package='wsl_joystick_bridge',
        executable='evdev_joy_node',
        name='evdev_joy_node',
        output='screen',
        parameters=[{
            'joy_topic': LaunchConfiguration('joy_topic'),
            'device_path': LaunchConfiguration('device_path'),
            'deadzone': LaunchConfiguration('deadzone'),
            'frame_id': LaunchConfiguration('frame_id'),
            'reconnect_seconds': LaunchConfiguration('reconnect_seconds'),
        }]
    )

    return LaunchDescription([
        joy_topic_arg,
        device_path_arg,
        deadzone_arg,
        frame_id_arg,
        reconnect_seconds_arg,
        joy_node
    ])
