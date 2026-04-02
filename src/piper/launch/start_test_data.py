from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    episode_idx_arg = DeclareLaunchArgument(
        'episode-idx',
        default_value='0',
        description='Index of the episode to load.'
    )
    # Define the node
    client_node = Node(
        package='piper',
        executable='piper_test_data',
        name='piper_test_data_node',
        parameters=[{
            'episode_idx': LaunchConfiguration('episode-idx'),
        }],
    )

    # Return the LaunchDescription
    return LaunchDescription([
        episode_idx_arg,
        client_node
    ])
