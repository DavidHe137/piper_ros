from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ── Arguments ──────────────────────────────────────────────────────────
    gripper_exist_arg = DeclareLaunchArgument(
        'gripper_exist',
        default_value='true',
        description='Whether a gripper is attached to the follower arm.'
    )

    # Default home pose — joints in radians (same units as ros2 topic echo joint_states).
    reset_joint_1_arg = DeclareLaunchArgument(
        'reset_joint_1', default_value='-0.02886982',
        description='Reset position for joint 1 (radians).'
    )
    reset_joint_2_arg = DeclareLaunchArgument(
        'reset_joint_2', default_value='1.0887149280000001',
        description='Reset position for joint 2 (radians).'
    )
    reset_joint_3_arg = DeclareLaunchArgument(
        'reset_joint_3', default_value='-1.17476618',
        description='Reset position for joint 3 (radians).'
    )
    reset_joint_4_arg = DeclareLaunchArgument(
        'reset_joint_4', default_value='-0.040696852000000006',
        description='Reset position for joint 4 (radians).'
    )
    reset_joint_5_arg = DeclareLaunchArgument(
        'reset_joint_5', default_value='1.194704672',
        description='Reset position for joint 5 (radians).'
    )
    reset_joint_6_arg = DeclareLaunchArgument(
        'reset_joint_6', default_value='-0.038743124000000004',
        description='Reset position for joint 6 (radians).'
    )
    reset_gripper_arg = DeclareLaunchArgument(
        'reset_gripper', default_value='0.0',
        description='Reset gripper opening in mm (0 = fully open).'
    )

    reset_speed_pct_arg = DeclareLaunchArgument(
        'reset_speed_pct',
        default_value='80',
        description='Speed percentage (0-100) used during the startup reset move.'
    )
    pos_scale_arg = DeclareLaunchArgument(
        'pos_scale',
        default_value='1.0',
        description='Scaling factor applied to Quest position deltas (1.0 = 1:1 mapping in metres).'
    )
    ori_weight_arg = DeclareLaunchArgument(
        'ori_weight',
        default_value='0.2',
        description='Orientation weight in the IK (0=position only, 0.2=position prioritised, 1.0=unweighted).'
    )
    speed_pct_arg = DeclareLaunchArgument(
        'speed_pct',
        default_value='30',
        description='Motion speed percentage (0-100) forwarded to the follower arm.'
    )
    control_hz_arg = DeclareLaunchArgument(
        'control_hz',
        default_value='30.0',
        description='Quest control loop rate in Hz.'
    )

    # ── Node ───────────────────────────────────────────────────────────────
    quest_teleop_node = Node(
        package='piper',
        executable='piper_quest_teleop',
        name='piper_quest_teleop_node',
        output='screen',
        parameters=[{
            'gripper_exist':  LaunchConfiguration('gripper_exist'),
            'reset_joint_1':  LaunchConfiguration('reset_joint_1'),
            'reset_joint_2':  LaunchConfiguration('reset_joint_2'),
            'reset_joint_3':  LaunchConfiguration('reset_joint_3'),
            'reset_joint_4':  LaunchConfiguration('reset_joint_4'),
            'reset_joint_5':  LaunchConfiguration('reset_joint_5'),
            'reset_joint_6':  LaunchConfiguration('reset_joint_6'),
            'reset_gripper':  LaunchConfiguration('reset_gripper'),
            'reset_speed_pct': LaunchConfiguration('reset_speed_pct'),
            'pos_scale':      LaunchConfiguration('pos_scale'),
            'ori_weight':     LaunchConfiguration('ori_weight'),
            'speed_pct':      LaunchConfiguration('speed_pct'),
            'control_hz':     LaunchConfiguration('control_hz'),
        }],
    )

    return LaunchDescription([
        gripper_exist_arg,
        reset_joint_1_arg,
        reset_joint_2_arg,
        reset_joint_3_arg,
        reset_joint_4_arg,
        reset_joint_5_arg,
        reset_joint_6_arg,
        reset_gripper_arg,
        reset_speed_pct_arg,
        pos_scale_arg,
        ori_weight_arg,
        speed_pct_arg,
        control_hz_arg,
        quest_teleop_node,
    ])
