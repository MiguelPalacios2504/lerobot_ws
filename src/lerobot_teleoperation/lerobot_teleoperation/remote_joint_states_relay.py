#!/usr/bin/env python3
"""
Puente LAN: reenvía joint_states locales del leader hacia un topic compartido en red.

Solo corre en la laptop. La Raspberry ejecuta el follower + mirror.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class RemoteJointStatesRelay(Node):
    def __init__(self):
        super().__init__('remote_joint_states_relay')

        self.declare_parameter('input_topic', '/leader/joint_states')
        self.declare_parameter('output_topic', '/teleop/joint_states')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub = self.create_publisher(JointState, output_topic, qos)
        self._sub = self.create_subscription(
            JointState,
            input_topic,
            self._on_joint_states,
            qos,
        )

        self.get_logger().info(
            f'Remote twin relay: {input_topic} -> {output_topic} (LAN)'
        )

    def _on_joint_states(self, msg: JointState):
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = RemoteJointStatesRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
