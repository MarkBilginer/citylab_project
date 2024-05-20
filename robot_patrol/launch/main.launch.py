from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='direction_service_server_node',
            name='direction_service',
            output='screen'
        ),
        Node(
        package='robot_patrol',
        executable='robot_patrol_with_service_client_node',
        name='robot_patrol_with_service_client',
        output='screen'
        )
    ])
