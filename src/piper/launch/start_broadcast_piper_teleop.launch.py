from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    can_port_arg = DeclareLaunchArgument(
        'can_port',
        default_value='can0',
        description='CAN port to be used by the Piper node.'
    )
    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='true',
        description='Automatically enable the Piper node.'
    )
    gripper_exist_arg = DeclareLaunchArgument(
        'gripper_exist',
        default_value='true',
        description='Whether the arm has gripper control.'
    )
    gripper_val_mutiple_arg = DeclareLaunchArgument(
        'gripper_val_mutiple',
        default_value='1',
        description='Gripper position multiplier.'
    )

    reset_joint1_arg = DeclareLaunchArgument('reset_joint1', default_value='0.005128536')
    reset_joint2_arg = DeclareLaunchArgument('reset_joint2', default_value='0.943266856')
    reset_joint3_arg = DeclareLaunchArgument('reset_joint3', default_value='-0.9962617280000001')
    reset_joint4_arg = DeclareLaunchArgument('reset_joint4', default_value='0.0')
    reset_joint5_arg = DeclareLaunchArgument('reset_joint5', default_value='0.0')
    reset_joint6_arg = DeclareLaunchArgument('reset_joint6', default_value='-0.04006886800000001')
    reset_gripper_arg = DeclareLaunchArgument('reset_gripper', default_value='0.0')

    reset_motion_speed_arg = DeclareLaunchArgument(
        'reset_motion_speed',
        default_value='30',
        description='Reset joint speed (1-100).'
    )
    reset_gripper_effort_arg = DeclareLaunchArgument(
        'reset_gripper_effort',
        default_value='1.0',
        description='Reset gripper effort in N.'
    )
    reset_hold_seconds_arg = DeclareLaunchArgument(
        'reset_hold_seconds',
        default_value='4.0',
        description='How long to keep sending the reset command.'
    )
    reset_command_hz_arg = DeclareLaunchArgument(
        'reset_command_hz',
        default_value='10.0',
        description='How fast to publish reset commands.'
    )
    wait_for_enable_timeout_arg = DeclareLaunchArgument(
        'wait_for_enable_timeout',
        default_value='10.0',
        description='Timeout waiting for enable state before reset.'
    )
    startup_delay_seconds_arg = DeclareLaunchArgument(
        'startup_delay_seconds',
        default_value='0.5',
        description='Delay before running startup reset.'
    )
    prompt_to_continue_arg = DeclareLaunchArgument(
        'prompt_to_continue',
        default_value='true',
        description='Pause broadcast and wait for Enter after reset.'
    )

    broadcast_teleop_node = Node(
        package='piper',
        executable='piper_broadcast_teleop',
        name='piper_broadcast_teleop',
        output='screen',
        parameters=[{
            'can_port': LaunchConfiguration('can_port'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'gripper_exist': LaunchConfiguration('gripper_exist'),
            'gripper_val_mutiple': LaunchConfiguration('gripper_val_mutiple'),
            'reset_joint1': LaunchConfiguration('reset_joint1'),
            'reset_joint2': LaunchConfiguration('reset_joint2'),
            'reset_joint3': LaunchConfiguration('reset_joint3'),
            'reset_joint4': LaunchConfiguration('reset_joint4'),
            'reset_joint5': LaunchConfiguration('reset_joint5'),
            'reset_joint6': LaunchConfiguration('reset_joint6'),
            'reset_gripper': LaunchConfiguration('reset_gripper'),
            'reset_motion_speed': LaunchConfiguration('reset_motion_speed'),
            'reset_gripper_effort': LaunchConfiguration('reset_gripper_effort'),
            'reset_hold_seconds': LaunchConfiguration('reset_hold_seconds'),
            'reset_command_hz': LaunchConfiguration('reset_command_hz'),
            'wait_for_enable_timeout': LaunchConfiguration('wait_for_enable_timeout'),
            'startup_delay_seconds': LaunchConfiguration('startup_delay_seconds'),
            'prompt_to_continue': LaunchConfiguration('prompt_to_continue'),
        }]
    )

    return LaunchDescription([
        can_port_arg,
        auto_enable_arg,
        gripper_exist_arg,
        gripper_val_mutiple_arg,
        reset_joint1_arg,
        reset_joint2_arg,
        reset_joint3_arg,
        reset_joint4_arg,
        reset_joint5_arg,
        reset_joint6_arg,
        reset_gripper_arg,
        reset_motion_speed_arg,
        reset_gripper_effort_arg,
        reset_hold_seconds_arg,
        reset_command_hz_arg,
        wait_for_enable_timeout_arg,
        startup_delay_seconds_arg,
        prompt_to_continue_arg,
        broadcast_teleop_node,
    ])
