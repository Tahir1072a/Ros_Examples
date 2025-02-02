from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_path

import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    gazebo_ros_server_path = os.path.join(get_package_share_path('gazebo_ros'), 'launch', 'gzserver.launch.py')
    gazebo_ros_client_path = os.path.join(get_package_share_path('gazebo_ros'), 'launch', 'gzclient.launch.py')
    
    urdf_path = os.path.join(get_package_share_path('robot_launch_package'), 'urdf', 'main.xacro')
    rviz_config_path = os.path.join(get_package_share_path('robot_launch_package'), 'rviz', 'urdf_config.rviz')

    robot_description_file = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_file}]
    )

    gazebo_ros_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "robot", "-x", "0", "-y", "0", "-z", "1"],
        output="screen"
    )

    # Gazebo Server Launch
    gazebo_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_ros_server_path)
    )

    # Gazebo Client Launch (Optional)
    gazebo_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_ros_client_path)
    )
    
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_server_launch,
        gazebo_client_launch,
        gazebo_ros_node,
        rviz2_node
    ])