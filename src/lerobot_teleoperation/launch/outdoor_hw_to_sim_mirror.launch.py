"""
Mirror a physical LeRobot arm (leader) to the arm mounted on OUTDOOR_ROBOT Gazebo sim.

Requires:
  - Terminal 1: ros2 launch bot_gazebo sim_swerve.launch.py use_lerobot:=true use_sim_time:=true
  - Terminal 2: ros2 launch lerobot_controller controller.launch.py \\
        is_sim:=false ns:=leader leader_only:=true uart_port:=/dev/ttyACM0
  - Terminal 3: this launch file
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SOURCE_ARM_JOINTS = [
    'limb1_to_base_link',
    'limb2_to_limb1',
    'limb3_to_limb2',
    'limb4_to_limb3',
    'limb5_to_limb4',
]
SOURCE_GRIPPER_JOINTS = ['limb6_to_limb5']


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('source_ns', default_value='leader'),
        DeclareLaunchArgument(
            'target_ns',
            default_value='',
            description='Empty for OUTDOOR_ROBOT sim (global namespace).',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('publish_deadband', default_value='0.002'),
        DeclareLaunchArgument('smoothing_alpha', default_value='0.92'),
        Node(
            package='lerobot_teleoperation',
            executable='mirror_joint_states_to_trajectory',
            name='outdoor_hw_to_sim_mirror',
            output='screen',
            parameters=[{
                'source_ns': LaunchConfiguration('source_ns'),
                'target_ns': LaunchConfiguration('target_ns'),
                'command_mode': 'trajectory',
                'arm_controller_name': 'lerobot_arm_controller',
                'gripper_controller_name': 'lerobot_gripper_controller',
                'publish_deadband': LaunchConfiguration('publish_deadband'),
                'smoothing_alpha': LaunchConfiguration('smoothing_alpha'),
                'trajectory_duration': 0.05,
                'arm_joints': SOURCE_ARM_JOINTS,
                'gripper_joints': SOURCE_GRIPPER_JOINTS,
                'joint_name_prefix': 'lerobot_',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
