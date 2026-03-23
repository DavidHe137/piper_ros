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
        default_value='0xAD',
        description='Speed percentage forwarded to the follower arm (1-100), 0xAD is high-follow.',
    )
    can_mode_timeout_arg = DeclareLaunchArgument(
        'can_mode_timeout',
        default_value='5.0',
        description='Seconds to wait for CAN mode confirmation after exiting teach mode.',
    )
    preset_speed_arg = DeclareLaunchArgument(
        'preset_speed',
        default_value='50',
        description='Speed percentage used when resetting follower to zero/preset (1-100). '
                    'Lower than move_speed to avoid sudden fast motion.',
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
            'preset_speed': LaunchConfiguration('preset_speed'),
        }],
    )

    return LaunchDescription([
        can_port_arg,
        gripper_exist_arg,
        move_speed_arg,
        can_mode_timeout_arg,
        preset_speed_arg,
        teleop_loop_node,
    ])
