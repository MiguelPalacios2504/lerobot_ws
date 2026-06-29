"""
Puente de red para twin remoto (solo laptop).

Reenvía /leader/joint_states -> /teleop/joint_states para que la Raspberry lo reciba por LAN.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topic',
            default_value='/leader/joint_states',
            description='joint_states del brazo líder local (USB en laptop)',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/teleop/joint_states',
            description='Topic publicado en LAN hacia la Raspberry',
        ),
        Node(
            package='lerobot_teleoperation',
            executable='remote_joint_states_relay',
            name='remote_joint_states_relay',
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
            }],
        ),
    ])
