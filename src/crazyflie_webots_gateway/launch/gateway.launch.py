from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    gateway = Node(
        package="crazyflie_webots_gateway",
        executable="crazyflie_webots_gateway",
        name="crazyflie_webots_gateway"
    )

    return LaunchDescription([
        gateway
    ])