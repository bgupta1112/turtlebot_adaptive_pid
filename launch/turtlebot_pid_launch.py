from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
	Node(
	    package = 'turtlesim',
            executable='turtlesim_node',
            name = 'turtlesim',
            output = 'screen'
            ),
        Node(
            package = 'turtlebot_adaptive_pid',
            executable='pid',
            name = 'pid_controller',
            output = 'screen' 
            )
	])
