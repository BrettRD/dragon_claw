#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

package_name = 'claw'


def generate_launch_description():

    # launch parameters
    urdf_filename = LaunchConfiguration('urdf_filename')
    urdf_filepath = LaunchConfiguration('urdf_filepath')

    claw_servo_config = LaunchConfiguration('claw_servo_config')
    claw_servo_config_filepath = LaunchConfiguration('claw_servo_config_filepath')

    joy_config = LaunchConfiguration('joy_config')
    joy_config_filepath = LaunchConfiguration('joy_config_filepath')


    # joystick input to gait synth
    joystick_launch_descriptions = [
        DeclareLaunchArgument(
            'joy_config',
            default_value='logi_f310',
            description='short name of the joystick config yaml'
        ),
        DeclareLaunchArgument(
            'joy_config_filepath',
            default_value=[
                TextSubstitution(text=os.path.join(
                    get_package_share_directory(package_name), 'config', '')),
                joy_config, TextSubstitution(text='.config.yaml')],
            description='full path to the joystick config yaml'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[joy_config_filepath]
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
            parameters=[joy_config_filepath]
        ),
    ]


    # URDF loader and forward kinematics
    urdf_launch_descriptions = [
        DeclareLaunchArgument('urdf_filename', default_value=package_name),
        DeclareLaunchArgument('urdf_filepath', default_value=[
            TextSubstitution(text=os.path.join(
                get_package_share_directory(package_name), 'urdf', '')),
            urdf_filename, TextSubstitution(text='.xacro')]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_filepath])
            }]
        ),
    ]

    # GUI control interface
    manual_control_launch_descriptions = [
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{}]),
    ]

    # gait synth and inverse kinematics
    synth_launch_descriptions = [
        Node(
            package=package_name,
            executable='gait_synth_node',
            name='gait_synth',
            output='screen',
            parameters=[{}]
        ),
        Node(
            package=package_name,
            executable='kinematics_node',
            name='kinematics',
            output='screen',
            parameters=[{}]
        ),
    ]

    # mix settings for a PWM output node
    hardware_launch_descriptions = [
        DeclareLaunchArgument(
            'claw_servo_config',
            default_value='claw_servo',
            description='short name of the claw motor config yaml'
        ),
        DeclareLaunchArgument(
            'claw_servo_config_filepath',
            default_value=[
                TextSubstitution(text=os.path.join(
                    get_package_share_directory(package_name), 'config', '')),
                claw_servo_config, TextSubstitution(text='.config.yaml')],
            description='full path to the claw motor config yaml'
        ),
        Node(
            package=package_name,
            executable='joint_map_node',
            name='joint_map',
            output='screen',
            parameters=[claw_servo_config_filepath]
        ),
    ]


    return LaunchDescription(
        urdf_launch_descriptions +
        # manual_control_launch_descriptions +
        joystick_launch_descriptions +
        synth_launch_descriptions +
        hardware_launch_descriptions
        )
