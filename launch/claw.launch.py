#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

package_name = 'claw'

def generate_launch_description():
    urdf_filename = LaunchConfiguration('urdf_filename')
    urdf_filepath = LaunchConfiguration('urdf_filepath')

    claw_servo_config = LaunchConfiguration('claw_servo_config')
    claw_servo_config_filepath = LaunchConfiguration('claw_servo_config_filepath')


    return LaunchDescription([
        DeclareLaunchArgument('urdf_filename', default_value='claw'),

        DeclareLaunchArgument('urdf_filepath', default_value=[
            TextSubstitution(text=os.path.join(
                get_package_share_directory(package_name), 'urdf', '')),
            urdf_filename, TextSubstitution(text='.xacro')]),


        DeclareLaunchArgument(
            'claw_servo_config',
            default_value='claw_servo',
            description='short name of the claw motor config yaml'
        ),

        DeclareLaunchArgument(
            'claw_servo_config_filepath',
            default_value=[
                TextSubstitution(text=os.path.join(
                    get_package_share_directory('claw'), 'config', '')),
                claw_servo_config, TextSubstitution(text='.config.yaml')],
            description='full path to the claw motor config yaml'
        ),



        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_filepath])
            }]),
        #Node(
        #    package='joint_state_publisher_gui',
        #    executable='joint_state_publisher_gui',
        #    name='joint_state_publisher_gui',
        #    output='screen',
        #    parameters=[{}]),
        Node(
            package='claw',
            executable='gait_synth_node',
            name='gait_synth',
            output='screen',
            parameters=[{}]),
        Node(
            package='claw',
            executable='kinematics_node',
            name='kinematics',
            output='screen',
            parameters=[{}]),
        Node(
            package='claw',
            executable='joint_map_node',
            name='joint_map',
            output='screen',
            parameters=[claw_servo_config_filepath]),
    ])
