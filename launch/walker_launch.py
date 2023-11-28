import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Tunable arguments
    ros_bag_choice = DeclareLaunchArgument('record_bag', default_value = TextSubstitution(text = "False"), choices = ['True', 'False'], description = "The argument that enables ros bag recording.")

    
    turtle_bot_world = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch'), '/turtlebot3_world.launch.py'])
    )

    walker = Node(
        package = 'simple_walker',
        executable = 'walker',
    )

    recorder = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('record_bag')),
            cmd=['ros2', 'bag', 'record', '-a', '-x', '/camera/*'],
        shell=True
    )

    return LaunchDescription([
        ros_bag_choice,
        turtle_bot_world,
        walker,
        recorder
    ])