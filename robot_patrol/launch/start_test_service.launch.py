from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_patrol',  # Replace with your actual package name
            executable='test_service_client_node',
            name='test_service',
            output='screen'
        )
    ])