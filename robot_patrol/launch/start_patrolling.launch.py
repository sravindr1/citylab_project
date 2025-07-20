from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    rviz_config = os.path.join(
        get_package_share_directory('robot_patrol'),
        'rviz',
        'patrol_config.rviz'
    )
    
    print(rviz_config)
    print(get_package_share_directory('robot_patrol'))

    return LaunchDescription([
    Node(
    package = "robot_patrol",
    executable = "robot_patrol_node",
    output ="screen",
    name = "robot_patrol_node" # name optional
    ),

  Node(
    package = "rviz2",
    executable = "rviz2",
    output ="screen",
    arguments=['-d', rviz_config],
    name = "rviz2" # name optional
    ),

    ])