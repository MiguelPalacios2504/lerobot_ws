from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARM_JOINTS = [
    'limb1_to_base_link',
    'limb2_to_limb1',
    'limb3_to_limb2',
    'limb4_to_limb3',
    'limb5_to_limb4',
]

GRIPPER_JOINTS = ['limb6_to_limb5']


def generate_launch_description():
    source_ns_arg = DeclareLaunchArgument('source_ns', default_value='leader')
    target_ns_arg = DeclareLaunchArgument(
        'target_ns',
        default_value='follower',
        description='Namespace del follower. Usa "/" para sin namespace (topics en /arm_controller/...).',
    )
    source_topic_arg = DeclareLaunchArgument(
        'source_joint_states_topic',
        default_value='',
        description='Topic absoluto del leader (ej. /teleop/joint_states para twin remoto).',
    )
    mode_arg = DeclareLaunchArgument('command_mode', default_value='forward')
    deadband_arg = DeclareLaunchArgument('publish_deadband', default_value='0.004')
    alpha_arg = DeclareLaunchArgument('smoothing_alpha', default_value='0.92')

    mirror = Node(
        package='lerobot_teleoperation',
        executable='mirror_joint_states_to_trajectory',
        name='mirror_joint_states_to_trajectory',
        output='screen',
        parameters=[{
            'source_ns': LaunchConfiguration('source_ns'),
            'target_ns': LaunchConfiguration('target_ns'),
            'source_joint_states_topic': LaunchConfiguration('source_joint_states_topic'),
            'command_mode': LaunchConfiguration('command_mode'),
            'publish_deadband': LaunchConfiguration('publish_deadband'),
            'smoothing_alpha': LaunchConfiguration('smoothing_alpha'),
            'arm_joints': ARM_JOINTS,
            'gripper_joints': GRIPPER_JOINTS,
        }],
    )

    return LaunchDescription([
        source_ns_arg,
        target_ns_arg,
        source_topic_arg,
        mode_arg,
        deadband_arg,
        alpha_arg,
        mirror,
    ])
