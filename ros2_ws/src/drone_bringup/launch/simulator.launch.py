from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    facehugger_node = Node(
        package="drone",
        executable="facehugger_simulator_node",
    )
    
    lidar_node = Node(
        package="lidar",
        executable="lidar_simulator_node",
    )

    ld = LaunchDescription()
    ld.add_action(facehugger_node)
    ld.add_action(lidar_node)
    return ld
