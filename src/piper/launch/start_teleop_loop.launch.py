from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    can_port_arg = DeclareLaunchArgument(
        'can_port',
        default_value='can0',
        description='CAN port for the teacher arm.',
    )
    gripper_exist_arg = DeclareLaunchArgument(
        'gripper_exist',
        default_value='true',
        description='Whether a gripper is attached to the teacher arm.',
    )
    move_speed_arg = DeclareLaunchArgument(
        'move_speed',
        default_value='30',
        description='Speed percentage forwarded to the follower arm (1-100).',
    )
    can_mode_timeout_arg = DeclareLaunchArgument(
        'can_mode_timeout',
        default_value='5.0',
        description='Seconds to wait for CAN mode confirmation after exiting teach mode.',
    )

    teleop_loop_node = Node(
        package='piper',
        executable='piper_teleop_loop',
        name='piper_teleop_loop',
        output='screen',
        parameters=[{
            'can_port': LaunchConfiguration('can_port'),
            'gripper_exist': LaunchConfiguration('gripper_exist'),
            'move_speed': LaunchConfiguration('move_speed'),
            'can_mode_timeout': LaunchConfiguration('can_mode_timeout'),
        }],
    )

    return LaunchDescription([
        can_port_arg,
        gripper_exist_arg,
        move_speed_arg,
        can_mode_timeout_arg,
        teleop_loop_node,
    ])
