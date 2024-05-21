from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    server_node = Node(
        package='robot_patrol',
        executable='direction_service_server_node',
        name='direction_service_server',
        output='screen'
    )
    
    client_node = Node(
        package='robot_patrol',
        executable='robot_patrol_with_service_client_node',
        name='robot_patrol_with_service_client',
        output='screen'
    )
    
    return LaunchDescription([
        server_node,
        TimerAction(
            period=5.0,  # Wait for 5 seconds before starting the client node
            actions=[client_node]
        )
    ])