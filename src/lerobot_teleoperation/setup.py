from setuptools import find_packages, setup

package_name = 'lerobot_teleoperation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/teleop_mirror.launch.py',
            'launch/dual_robot_teleop.launch.py',
            'launch/outdoor_hw_to_sim_mirror.launch.py',
            'launch/teleop_remote_relay.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/cyclonedds_laptop.xml',
            'config/cyclonedds_pi.xml',
        ]),
        ('share/' + package_name + '/env', [
            'env/remote_laptop.bash',
            'env/remote_pi.bash',
        ]),
        ('share/' + package_name + '/rviz', ['rviz/teleop_dual.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mi',
    maintainer_email='a20203884@pucp.edu.pe',
    description='Mirror joint states from one robot to another via JointTrajectory.',
    license='Apache-2.0',
    tests_require=['pytest'],
        entry_points={
        'console_scripts': [
            'mirror_joint_states_to_trajectory = lerobot_teleoperation.mirror_joint_states_to_trajectory:main',
            'remote_joint_states_relay = lerobot_teleoperation.remote_joint_states_relay:main',
        ],
    },
)
