"""
Simulation / visualisation launch for Quest teleoperation.

Starts:
  - robot_state_publisher  (subscribes to /joint_states, drives the URDF model)
  - rviz2                  (visualises the model)
  - piper_quest_teleop     (publishes /joint_states from Quest controller input)

Does NOT start:
  - piper_ctrl_single_node  (no CAN / no real arm required)
  - joint_state_publisher   (would conflict with the Quest teleop publisher)
"""
import os

from ament_index_python.packages import get_package_share_directory, get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ── Paths ──────────────────────────────────────────────────────────────
    piper_desc_path = get_package_share_path('piper_description')
    default_urdf    = str(piper_desc_path / 'urdf/piper_description.urdf')
    default_rviz    = str(piper_desc_path / 'rviz/piper_ctrl.rviz')

    # ── Arguments ──────────────────────────────────────────────────────────
    model_arg = DeclareLaunchArgument(
        'model', default_value=default_urdf,
        description='Absolute path to robot URDF file.'
    )
    rviz_arg = DeclareLaunchArgument(
        'rvizconfig', default_value=default_rviz,
        description='Absolute path to RViz config file.'
    )
    gripper_exist_arg = DeclareLaunchArgument(
        'gripper_exist', default_value='true',
        description='Whether a gripper is attached.'
    )
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
        description='Reset gripper opening in mm.'
    )
    reset_speed_pct_arg = DeclareLaunchArgument(
        'reset_speed_pct', default_value='80',
        description='Speed % used during startup reset (ignored in sim, kept for API parity).'
    )
    pos_scale_arg = DeclareLaunchArgument(
        'pos_scale', default_value='1.0',
        description='Multiplier on Quest position deltas.'
    )
    ori_weight_arg = DeclareLaunchArgument(
        'ori_weight', default_value='0.2',
        description='Orientation weight in the IK (0=position only, 0.2=position prioritised, 1.0=unweighted).'
    )
    speed_pct_arg = DeclareLaunchArgument(
        'speed_pct', default_value='30',
        description='Speed % during teleoperation (ignored in sim, kept for API parity).'
    )
    control_hz_arg = DeclareLaunchArgument(
        'control_hz', default_value='30.0',
        description='Quest control loop rate in Hz.'
    )

    # ── Nodes ───────────────────────────────────────────────────────────────
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    quest_teleop_node = Node(
        package='piper',
        executable='piper_quest_teleop',
        name='piper_quest_teleop_node',
        output='screen',
        parameters=[{
            'gripper_exist':   LaunchConfiguration('gripper_exist'),
            'reset_joint_1':   LaunchConfiguration('reset_joint_1'),
            'reset_joint_2':   LaunchConfiguration('reset_joint_2'),
            'reset_joint_3':   LaunchConfiguration('reset_joint_3'),
            'reset_joint_4':   LaunchConfiguration('reset_joint_4'),
            'reset_joint_5':   LaunchConfiguration('reset_joint_5'),
            'reset_joint_6':   LaunchConfiguration('reset_joint_6'),
            'reset_gripper':   LaunchConfiguration('reset_gripper'),
            'reset_speed_pct': LaunchConfiguration('reset_speed_pct'),
            'pos_scale':       LaunchConfiguration('pos_scale'),
            'ori_weight':      LaunchConfiguration('ori_weight'),
            'speed_pct':       LaunchConfiguration('speed_pct'),
            'control_hz':      LaunchConfiguration('control_hz'),
        }],
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
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
        robot_state_publisher_node,
        rviz_node,
        quest_teleop_node,
    ])
