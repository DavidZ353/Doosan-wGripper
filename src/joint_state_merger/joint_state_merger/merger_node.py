#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateMerger(Node):
    def __init__(self):
        super().__init__('joint_state_merger')

        # Parameter deklarieren
        self.declare_parameter('mode', 'virtual')
        mode = self.get_parameter('mode').get_parameter_value().string_value

        # Gripper ID basierend auf mode setzen
        if mode == 'virtual':
            self.gripper_id = 'EGK_40_M_B_1'
        else:
            self.gripper_id = 'EGU_60_M_B_1'

        self.get_logger().info(f"Gripper model is: {self.gripper_id}")

        self.joint_states_robot = JointState()
        self.joint_states_gripper = JointState()

        self.sub_robot = self.create_subscription(
            JointState,
            '/joint_states',
            self.robot_callback,
            10)

        self.sub_gripper = self.create_subscription(
            JointState,
            f'/schunk/driver/{self.gripper_id}/joint_states',
            self.gripper_callback,
            10)

        self.pub = self.create_publisher(JointState, '/merged_joint_states', 10)

        self.timer = self.create_timer(0.01, self.publish_merged_joint_states)  # 100 Hz

    def robot_callback(self, msg):
        self.joint_states_robot = msg

    def gripper_callback(self, msg):
        self.joint_states_gripper = msg

    def publish_merged_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

         # Base joint data
        msg.name = list(self.joint_states_robot.name)
        msg.position = list(self.joint_states_robot.position)
        msg.velocity = list(self.joint_states_robot.velocity)
        msg.effort = list(self.joint_states_robot.effort)

        # Gripper transformation
        if self.gripper_id in self.joint_states_gripper.name:
            idx = self.joint_states_gripper.name.index(self.gripper_id)
            pos = self.joint_states_gripper.position[idx] / 2.0
            vel = self.joint_states_gripper.velocity[idx] / 2.0 if self.joint_states_gripper.velocity else 0.0
            eff = self.joint_states_gripper.effort[idx] / 2.0 if self.joint_states_gripper.effort else 0.0

            # Add left and right finger joints
            msg.name.extend(["left_finger_joint", "right_finger_joint"])
            msg.position.extend([pos, pos])
            msg.velocity.extend([vel, vel])
            msg.effort.extend([eff, eff])
        else:
            self.get_logger().warn(f"Gripper joint '{self.gripper_id}' not found in message")


        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
