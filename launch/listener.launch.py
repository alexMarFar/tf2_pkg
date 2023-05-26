# Copyright (C) - All Rights Reserved
#
# Written by Alejandra Martínez Fariña <alejandra.mf23be@gmail.com>
# Licensed under the Apache License, Version 2.0

import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define the topic name launch argument
    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='target_frame',
        description='Name of the target frame'
    )
    
    source_frame_arg = DeclareLaunchArgument(
        'source_frame',
        default_value='source_frame',
        description='Name of the source frame'
    )

    # Launch the C++ executable
    listener_cpp_node = launch_ros.actions.Node(
        package='tf2_pkg',
        executable='listener_cpp',
        output='screen',
        parameters=[{'target_frame': LaunchConfiguration('target_frame')},
        {'source_frame': LaunchConfiguration('source_frame')}]
    )

    # Launch the Python executable
    listener_python_node = launch_ros.actions.Node(
        package='tf2_pkg',
        executable='listener_python.py',
        output='screen',
        emulate_tty=True,
        parameters=[{'target_frame': LaunchConfiguration('target_frame')},
        {'source_frame': LaunchConfiguration('source_frame')}]
    )

    # Create the launch description
    ld = launch.LaunchDescription()

    # Add the launch arguments and nodes to the launch description
    ld.add_action(target_frame_arg)
    ld.add_action(source_frame_arg)
    ld.add_action(listener_cpp_node)
    ld.add_action(listener_python_node)

    return ld
