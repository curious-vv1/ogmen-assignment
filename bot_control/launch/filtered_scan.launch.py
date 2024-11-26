from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Start the Python filter_scan node
        Node(
            package='bot_control',  
            executable='reading_laser.py',  
            output='screen',
        ),
        # Start RViz
        ExecuteProcess(
            cmd=[
                'rviz2',
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('bot_control'),  
                    'config',
                    'filter_scan.rviz'  
                ])
            ],
            output='screen',
        ),
    ])
