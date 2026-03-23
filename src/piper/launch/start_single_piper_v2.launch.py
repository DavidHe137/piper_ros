from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


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
        description='gripper'
    )
    gripper_val_mutiple_arg = DeclareLaunchArgument(
        'gripper_val_mutiple',
        default_value='1',
        description='gripper'
    )
    mit_kp_arg = DeclareLaunchArgument(
        'mit_kp',
        default_value='10.0',
        description='MIT proportional gain applied to all joints (per-joint overrides: joint1_kp … joint6_kp).'
    )
    mit_kd_arg = DeclareLaunchArgument(
        'mit_kd',
        default_value='0.8',
        description='MIT derivative gain applied to all joints (per-joint overrides: joint1_kd … joint6_kd).'
    )

    piper_node_v2 = Node(
        package='piper',
        executable='piper_single_ctrl_v2',
        name='piper_ctrl_single_node_v2',
        output='screen',
        parameters=[{
            'can_port': LaunchConfiguration('can_port'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'gripper_val_mutiple': LaunchConfiguration('gripper_val_mutiple'),
            'gripper_exist': LaunchConfiguration('gripper_exist'),
            'joint1_kp': LaunchConfiguration('mit_kp'),
            'joint2_kp': LaunchConfiguration('mit_kp'),
            'joint3_kp': LaunchConfiguration('mit_kp'),
            'joint4_kp': LaunchConfiguration('mit_kp'),
            'joint5_kp': LaunchConfiguration('mit_kp'),
            'joint6_kp': LaunchConfiguration('mit_kp'),
            'joint1_kd': LaunchConfiguration('mit_kd'),
            'joint2_kd': LaunchConfiguration('mit_kd'),
            'joint3_kd': LaunchConfiguration('mit_kd'),
            'joint4_kd': LaunchConfiguration('mit_kd'),
            'joint5_kd': LaunchConfiguration('mit_kd'),
            'joint6_kd': LaunchConfiguration('mit_kd'),
        }],
        remappings=[
            ('joint_ctrl_single', '/joint_states'),
        ]
    )

    return LaunchDescription([
        can_port_arg,
        auto_enable_arg,
        gripper_exist_arg,
        gripper_val_mutiple_arg,
        mit_kp_arg,
        mit_kd_arg,
        piper_node_v2,
    ])
