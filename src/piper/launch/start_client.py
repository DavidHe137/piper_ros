from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define the node
    client_node = Node(
        package='piper',
        executable='piper_client',
        name='piper_client_node'
    )

    # Return the LaunchDescription
    return LaunchDescription([
        client_node
    ])
