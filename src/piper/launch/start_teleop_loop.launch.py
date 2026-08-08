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
    top_image_topic_arg = DeclareLaunchArgument(
        'top_image_topic',
        default_value='/camera/intel_realsense_d435i_top/color/image_raw',
        description='ROS topic for the top camera color image.',
    )
    wrist_image_topic_arg = DeclareLaunchArgument(
        'wrist_image_topic',
        default_value='/camera/intel_realsense_d435i_wrist/color/image_raw',
        description='ROS topic for the wrist camera color image.',
    )
    viewer_height_arg = DeclareLaunchArgument(
        'viewer_height',
        default_value='480',
        description='Height in pixels for each camera panel in the viewer window.',
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
            'top_image_topic': LaunchConfiguration('top_image_topic'),
            'wrist_image_topic': LaunchConfiguration('wrist_image_topic'),
            'viewer_height': LaunchConfiguration('viewer_height'),
        }],
    )

    return LaunchDescription([
        can_port_arg,
        gripper_exist_arg,
        move_speed_arg,
        can_mode_timeout_arg,
        preset_speed_arg,
        top_image_topic_arg,
        wrist_image_topic_arg,
        viewer_height_arg,
        teleop_loop_node,
    ])
