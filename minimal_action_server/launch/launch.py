from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch.substitutions import Command,FindExecutable,PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
	node_params = PathJoinSubstitution([FindPackageShare('minimal_action_server'), 'config', 'minimal_action_server.yaml'])

	minimal_action_server_node = Node(
		package='minimal_action_server',
		executable='minimal_action_server_node',
		parameters=[node_params],
		output='log'
	)

	return LaunchDescription([minimal_action_server_node])