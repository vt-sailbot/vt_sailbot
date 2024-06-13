from launch import LaunchDescription
from launch_ros.actions import Node

# VERY MUCH IN DEVELOPMENT THIS IS JUST A TEMPLATE
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',

            respawn=True, 
            respawn_delay=2.0,
            

            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ],
        )
    ])
