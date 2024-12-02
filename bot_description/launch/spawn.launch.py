from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_path


def generate_launch_description():

    # Paths to URDF and RViz configuration
    urdf_path = os.path.join(get_package_share_path('bot_description'),
                             'urdf', 'bot.urdf.xacro')
    rviz_config_path = os.path.join(get_package_share_path('bot_description'),
                                    'rviz', 'config_urdf.rviz')
    gazebo_ros_path = get_package_share_path('gazebo_ros')

    # Robot description parameter
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    # joint_state_publisher_gui_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui"
    # )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    # Include Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_path, 'launch', 'gazebo.launch.py')
        )
    )

    # Spawn robot entity in Gazebo
    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'bot'
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_launch,
        spawn_entity_node,
        rviz2_node
    ])
