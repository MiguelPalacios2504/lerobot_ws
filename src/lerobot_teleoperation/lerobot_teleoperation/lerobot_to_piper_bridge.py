#!/usr/bin/env python3
"""
Map LeRobot leader joint_states to Piper joint commands.

Arm mapping:
  limb1 -> joint1, limb2 -> joint2, limb3 -> joint3,
  limb4 -> joint5, limb5 -> joint6, joint4 held fixed.

Gripper mapping (linear):
  limb6 90 deg (closed) -> gripper 0.0 m
  limb6 180 deg (open)   -> gripper 0.04 m  (with gripper_val_mutiple:=2 on Piper)
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class LerobotToPiperBridge(Node):
    def __init__(self):
        super().__init__('lerobot_to_piper_bridge')

        self.declare_parameter('source_topic', '/leader/joint_states')
        self.declare_parameter('target_topic', '/joint_states')
        self.declare_parameter('lerobot_joints', [
            'limb1_to_base_link',
            'limb2_to_limb1',
            'limb3_to_limb2',
            'limb4_to_limb3',
            'limb5_to_limb4',
        ])
        self.declare_parameter('piper_joints', [
            'joint1', 'joint2', 'joint3', 'joint5', 'joint6',
        ])
        self.declare_parameter('joint4_fixed', 0.0)
        self.declare_parameter('joint_signs', [1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('joint_offsets', [0.0, 0.0, 0.0, 0.0, 0.0])

        self.declare_parameter('enable_gripper', True)
        self.declare_parameter('lerobot_gripper_joint', 'limb6_to_limb5')
        self.declare_parameter('lerobot_gripper_deg_min', 90.0)
        self.declare_parameter('lerobot_gripper_deg_max', 180.0)
        self.declare_parameter('piper_gripper_min', 0.0)
        self.declare_parameter('piper_gripper_max', 0.04)
        self.declare_parameter('gripper_effort', 1.0)

        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('publish_deadband', 0.002)
        self.declare_parameter('gripper_deadband', 0.0005)
        self.declare_parameter('smoothing_alpha', 0.85)
        self.declare_parameter('motion_velocity_percent', 30.0)

        self.source_topic = self.get_parameter('source_topic').value
        self.target_topic = self.get_parameter('target_topic').value
        self.lerobot_joints = list(self.get_parameter('lerobot_joints').value)
        self.piper_mapped_joints = list(self.get_parameter('piper_joints').value)
        self.joint4_fixed = float(self.get_parameter('joint4_fixed').value)
        self.joint_signs = [float(v) for v in self.get_parameter('joint_signs').value]
        self.joint_offsets = [float(v) for v in self.get_parameter('joint_offsets').value]

        self.enable_gripper = bool(self.get_parameter('enable_gripper').value)
        self.lerobot_gripper_joint = self.get_parameter('lerobot_gripper_joint').value
        self.lerobot_gripper_rad_min = math.radians(
            float(self.get_parameter('lerobot_gripper_deg_min').value))
        self.lerobot_gripper_rad_max = math.radians(
            float(self.get_parameter('lerobot_gripper_deg_max').value))
        self.piper_gripper_min = float(self.get_parameter('piper_gripper_min').value)
        self.piper_gripper_max = float(self.get_parameter('piper_gripper_max').value)
        self.gripper_effort = float(self.get_parameter('gripper_effort').value)

        self.publish_deadband = float(self.get_parameter('publish_deadband').value)
        self.gripper_deadband = float(self.get_parameter('gripper_deadband').value)
        self.smoothing_alpha = float(self.get_parameter('smoothing_alpha').value)
        self.motion_velocity_percent = float(self.get_parameter('motion_velocity_percent').value)

        span = self.lerobot_gripper_rad_max - self.lerobot_gripper_rad_min
        if abs(span) < 1e-9:
            raise ValueError('lerobot_gripper_deg_max must differ from lerobot_gripper_deg_min')

        if len(self.lerobot_joints) != len(self.piper_mapped_joints):
            raise ValueError('lerobot_joints and piper_joints must have the same length')
        if len(self.joint_signs) != len(self.lerobot_joints):
            raise ValueError('joint_signs length must match lerobot_joints')
        if len(self.joint_offsets) != len(self.lerobot_joints):
            raise ValueError('joint_offsets length must match lerobot_joints')

        self._arm_joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
        ]
        self._latest_leader = None
        self._smooth_mapped = None
        self._smooth_gripper = None
        self._last_sent_arm = None
        self._last_sent_gripper = None

        self.pub = self.create_publisher(JointState, self.target_topic, 10)
        self.sub = self.create_subscription(
            JointState, self.source_topic, self._on_leader_js, 10)

        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / max(rate_hz, 1.0), self._publish)

        self.get_logger().info(
            f'LeRobot -> Piper bridge: {self.source_topic} -> {self.target_topic}'
        )
        for src, dst, sign in zip(
                self.lerobot_joints, self.piper_mapped_joints, self.joint_signs):
            self.get_logger().info(f'  {src} -> {dst} (sign={sign:+.0f})')
        self.get_logger().info(f'  joint4 fixed at {self.joint4_fixed:.4f} rad')
        if self.enable_gripper:
            self.get_logger().info(
                f'  {self.lerobot_gripper_joint} '
                f'[{self.lerobot_gripper_rad_min:.3f}, {self.lerobot_gripper_rad_max:.3f}] rad '
                f'-> gripper [{self.piper_gripper_min:.3f}, {self.piper_gripper_max:.3f}] m'
            )

    def _on_leader_js(self, msg: JointState):
        if not msg.name or not msg.position:
            return
        if len(msg.name) != len(msg.position):
            return
        self._latest_leader = {n: p for n, p in zip(msg.name, msg.position)}

    @staticmethod
    def _smooth_scalar(prev, raw, alpha):
        if prev is None:
            return raw
        return alpha * raw + (1.0 - alpha) * prev

    @staticmethod
    def _smooth_list(prev, raw, alpha):
        if prev is None:
            return list(raw)
        return [alpha * r + (1.0 - alpha) * p for r, p in zip(raw, prev)]

    def _map_gripper(self):
        if not self.enable_gripper:
            return None
        if self._latest_leader is None:
            return None
        if self.lerobot_gripper_joint not in self._latest_leader:
            return None

        limb6 = self._latest_leader[self.lerobot_gripper_joint]
        span = self.lerobot_gripper_rad_max - self.lerobot_gripper_rad_min
        t = (limb6 - self.lerobot_gripper_rad_min) / span
        t = max(0.0, min(1.0, t))
        raw = self.piper_gripper_min + t * (
            self.piper_gripper_max - self.piper_gripper_min)
        self._smooth_gripper = self._smooth_scalar(
            self._smooth_gripper, raw, self.smoothing_alpha)
        return self._smooth_gripper

    def _build_arm_positions(self):
        if self._latest_leader is None:
            return None
        if not all(j in self._latest_leader for j in self.lerobot_joints):
            return None

        mapped = []
        for joint_name, sign, offset in zip(
                self.lerobot_joints, self.joint_signs, self.joint_offsets):
            mapped.append(sign * self._latest_leader[joint_name] + offset)

        self._smooth_mapped = self._smooth_list(
            self._smooth_mapped, mapped, self.smoothing_alpha)

        positions = {
            'joint1': 0.0,
            'joint2': 0.0,
            'joint3': 0.0,
            'joint4': self.joint4_fixed,
            'joint5': 0.0,
            'joint6': 0.0,
        }
        for piper_name, value in zip(self.piper_mapped_joints, self._smooth_mapped):
            positions[piper_name] = value

        return [positions[name] for name in self._arm_joint_names]

    def _publish(self):
        arm_positions = self._build_arm_positions()
        gripper = self._map_gripper()
        if arm_positions is None:
            return
        if self.enable_gripper and gripper is None:
            return

        arm_changed = self._last_sent_arm is None or any(
            abs(a - b) > self.publish_deadband
            for a, b in zip(self._last_sent_arm, arm_positions))
        gripper_changed = (
            not self.enable_gripper
            or self._last_sent_gripper is None
            or abs(gripper - self._last_sent_gripper) > self.gripper_deadband
        )
        if not arm_changed and not gripper_changed:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.enable_gripper:
            msg.name = self._arm_joint_names + ['gripper']
            msg.position = arm_positions + [gripper]
            msg.velocity = [0.0] * 6 + [self.motion_velocity_percent]
            msg.effort = [0.0] * 6 + [self.gripper_effort]
        else:
            msg.name = self._arm_joint_names
            msg.position = arm_positions
            msg.velocity = [0.0] * 6
            msg.effort = []

        self.pub.publish(msg)
        self._last_sent_arm = list(arm_positions)
        self._last_sent_gripper = gripper


def main():
    rclpy.init()
    node = LerobotToPiperBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()