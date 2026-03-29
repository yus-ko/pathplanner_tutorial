import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('apf_pathplanner_tutorial')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'apf_pathplanner_tutorial.rviz')

    return LaunchDescription([
        Node(
            package='apf_pathplanner_tutorial',
            executable='apf_pathplanner_tutorial',
            name='apf_pathplanner_tutorial',
            namespace='tutorial',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
