from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    power_monitor_node = Node(
        package='roberto_controller',
        executable='power_monitor',
    )

    i2c_host_node = Node(
        package='roberto_controller',
        executable='i2c_host',
    )

    ld.add_action(power_monitor_node)
    ld.add_action(i2c_host_node)

    return ld

    