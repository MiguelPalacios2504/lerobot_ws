#!/usr/bin/env python3
"""
Teleop baja latencia: reenvía posiciones del líder al follower en cada joint_states.
Modo forward_command (sin joint_trajectory_controller) para respuesta inmediata.
"""
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MirrorNode(Node):
    def __init__(self):
        super().__init__('mirror_joint_states_to_trajectory')

        self.declare_parameter('source_ns', 'leader')
        self.declare_parameter('target_ns', 'follower')
        self.declare_parameter('arm_controller_name', 'arm_controller')
        self.declare_parameter('gripper_controller_name', 'gripper_controller')
        self.declare_parameter('command_mode', 'forward')
        self.declare_parameter('publish_deadband', 0.004)
        self.declare_parameter('smoothing_alpha', 0.92)
        self.declare_parameter('trajectory_duration', 0.05)
        self.declare_parameter('arm_joints', [
            'limb1_to_base_link',
            'limb2_to_limb1',
            'limb3_to_limb2',
            'limb4_to_limb3',
            'limb5_to_limb4',
        ])
        self.declare_parameter('gripper_joints', ['limb6_to_limb5'])
        self.declare_parameter(
            'source_joint_states_topic',
            '',
            ParameterDescriptor(
                description='Topic absoluto de joint_states del leader. '
                            'Si está vacío, usa /<source_ns>/joint_states.',
            ),
        )
        self.declare_parameter(
            'joint_name_prefix',
            '',
            ParameterDescriptor(
                description='Prepended to source joint names for follower trajectories (e.g. lerobot_).',
            ),
        )

        self.source_ns = self.get_parameter('source_ns').get_parameter_value().string_value.strip('/')
        self.target_ns = self.get_parameter('target_ns').get_parameter_value().string_value.strip('/')
        self.arm_controller = self.get_parameter('arm_controller_name').get_parameter_value().string_value
        self.gripper_controller = self.get_parameter('gripper_controller_name').get_parameter_value().string_value
        self.command_mode = self.get_parameter('command_mode').get_parameter_value().string_value
        self.publish_deadband = float(self.get_parameter('publish_deadband').value)
        self.smoothing_alpha = float(self.get_parameter('smoothing_alpha').value)
        self.trajectory_duration = float(self.get_parameter('trajectory_duration').value)
        self.arm_joints = list(self.get_parameter('arm_joints').value)
        self.gripper_joints = list(self.get_parameter('gripper_joints').value)
        joint_prefix = self.get_parameter('joint_name_prefix').get_parameter_value().string_value
        if joint_prefix:
            self.target_arm_joints = [f'{joint_prefix}{j}' for j in self.arm_joints]
            self.target_gripper_joints = [f'{joint_prefix}{j}' for j in self.gripper_joints]
        else:
            self.target_arm_joints = self.arm_joints
            self.target_gripper_joints = self.gripper_joints

        source_topic = (
            self.get_parameter('source_joint_states_topic')
            .get_parameter_value().string_value.strip()
        )
        if source_topic:
            self._source_topic = (
                source_topic if source_topic.startswith('/') else f'/{source_topic}'
            )
        else:
            self._source_topic = self._ns_topic(self.source_ns, 'joint_states')

        self._smooth_arm = None
        self._smooth_gripper = None
        self._sent_arm = None
        self._sent_gripper = None

        self.sub = self.create_subscription(
            JointState,
            self._source_topic,
            self._on_js,
            10,
        )

        if self.command_mode == 'forward':
            self.arm_pub = self.create_publisher(
                Float64MultiArray,
                self._ns_topic(self.target_ns, f'{self.arm_controller}/commands'),
                10,
            )
            self.gripper_pub = self.create_publisher(
                Float64MultiArray,
                self._ns_topic(self.target_ns, f'{self.gripper_controller}/commands'),
                10,
            )
        else:
            self.arm_pub = self.create_publisher(
                JointTrajectory,
                self._ns_topic(self.target_ns, f'{self.arm_controller}/joint_trajectory'),
                10,
            )
            self.gripper_pub = self.create_publisher(
                JointTrajectory,
                self._ns_topic(self.target_ns, f'{self.gripper_controller}/joint_trajectory'),
                10,
            )

        self.get_logger().info(
            f'Teleop directo [{self.command_mode}]: '
            f'{self._source_topic} -> '
            f'{self._ns_topic(self.target_ns, "{arm,gripper}")} '
            f'(deadband={self.publish_deadband:.4f} rad)'
        )

    @staticmethod
    def _ns_topic(ns: str, suffix: str) -> str:
        ns = ns.strip('/')
        return f'/{ns}/{suffix}' if ns else f'/{suffix}'

    @staticmethod
    def _smooth(prev, raw, alpha):
        if prev is None:
            return list(raw)
        return [alpha * r + (1.0 - alpha) * p for r, p in zip(raw, prev)]

    @staticmethod
    def _changed(prev, cur, deadband):
        if prev is None:
            return True
        return any(abs(a - b) > deadband for a, b in zip(prev, cur))

    def _publish_forward(self, publisher, positions):
        msg = Float64MultiArray()
        msg.data = list(positions)
        publisher.publish(msg)

    def _publish_trajectory(self, publisher, joint_names, positions):
        traj = JointTrajectory()
        traj.joint_names = joint_names
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        duration_ns = int(max(0.02, self.trajectory_duration) * 1e9)
        pt.time_from_start.sec = duration_ns // 1_000_000_000
        pt.time_from_start.nanosec = duration_ns % 1_000_000_000
        traj.points.append(pt)
        publisher.publish(traj)

    def _maybe_send(self, publisher, joint_names, raw, smooth_attr, sent_attr):
        smooth = self._smooth(
            getattr(self, smooth_attr),
            raw,
            self.smoothing_alpha,
        )
        setattr(self, smooth_attr, smooth)

        sent = getattr(self, sent_attr)
        if not self._changed(sent, smooth, self.publish_deadband):
            return

        if self.command_mode == 'forward':
            self._publish_forward(publisher, smooth)
        else:
            self._publish_trajectory(publisher, joint_names, smooth)

        setattr(self, sent_attr, list(smooth))

    def _on_js(self, msg: JointState):
        if not msg.name or not msg.position or len(msg.name) != len(msg.position):
            return

        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}

        if all(j in name_to_pos for j in self.arm_joints):
            raw = [name_to_pos[j] for j in self.arm_joints]
            self._maybe_send(
                self.arm_pub,
                self.target_arm_joints,
                raw,
                '_smooth_arm',
                '_sent_arm',
            )

        if all(j in name_to_pos for j in self.gripper_joints):
            raw = [name_to_pos[j] for j in self.gripper_joints]
            self._maybe_send(
                self.gripper_pub,
                self.target_gripper_joints,
                raw,
                '_smooth_gripper',
                '_sent_gripper',
            )


def main():
    rclpy.init()
    node = MirrorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
