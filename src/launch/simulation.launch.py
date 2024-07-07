from launch import LaunchDescription
from launch_ros.actions import Node

# VERY MUCH IN DEVELOPMENT THIS IS JUST A TEMPLATE
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autopilot',
            executable='autopilot',
            name='autopilot',

            respawn=True, 
            respawn_delay=2.0,
            output="log"
        ),
        
        
        Node(
            package='autopilot',
            executable='telemetry',
            name='telemetry',

            respawn=True, 
            respawn_delay=2.0,
            output="log"
        ),
        
        
        Node(
            package='simulation',
            executable='simulation',
            name='simulation',

            respawn=True, 
            respawn_delay=2.0,
            output="log"
        )
    ])