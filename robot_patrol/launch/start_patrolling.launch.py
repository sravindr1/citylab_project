from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    
    
    return LaunchDescription([
    Node(
    package = "robot_patrol",
    executable = "robot_patrol_node",
    output ="screen",
    name = "robot_patrol_node" # name optional
    ),
    ])