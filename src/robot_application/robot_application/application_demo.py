#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from schunk_gripper_interfaces.srv import MoveToAbsolutePosition
from std_srvs.srv import Trigger
import time

from dsr_msgs2.srv import (
    MoveJoint,
    MoveLine,
)

class ApplicationDemo(Node):
    def __init__(self):
        super().__init__('integration_node')
        self.get_logger().info('Integration node started!')

        # Create service clients for move robot
        self.cli_movej = self.create_client(MoveJoint, '/motion/move_joint')
        self.cli_movel = self.create_client(MoveLine, '/motion/move_line')

        # wait for services to be available
        self.get_logger().info('Waiting for motion services...')
        self.cli_movej.wait_for_service()
        self.cli_movel.wait_for_service()
        self.get_logger().info('All motion services are available.')

        # Create service client for move gripper
        self.client = self.create_client(MoveToAbsolutePosition, '/schunk/driver/EGU_60_M_B_1/move_to_absolute_position')  
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for service to become available...')

        # Create service client for acknowledge gripper
        self.ack_client = self.create_client(Trigger, '/schunk/driver/EGU_60_M_B_1/acknowledge')
        while not self.ack_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for acknowledge service...')

        self.subscription = self.create_subscription(
            JointState,
            '/schunk/driver/EGU_60_M_B_1/joint_states',
            self.joint_state_callback,
            10
        )

        # Create a publisher for the /joint_states topic
        # This is the standard topic where robot joint states are published
        self.joint_state_pub = self.create_publisher(JointState, '/schunk/driver/EGU_60_M_B_1/joint_states', 10)

        # Set the publishing rate (e.g., 10 Hz)
        self.timer = self.create_timer(0.1, self.publish_gripper_state) # 0.1 seconds = 10 Hz

        self.get_logger().info("Gripper joint publisher started.")

        # Define your gripper joint names
        # IMPORTANT: Replace 'gripper_finger_joint' with the actual name(s) of your gripper joint(s)
        # from your URDF. If you have multiple joints for the gripper, list them here.
        self.gripper_joint_names = ['left_finger_joint', 'right_finger_joint'] # Example: ['left_finger_joint', 'right_finger_joint']

        # Define target positions for opening and closing the gripper
        # Adjust these values based on your gripper's joint limits
        self.open_position = 0.0  # Fully open (or a value close to it)
        self.closed_position = 0.06 # Fully closed (or a value close to it) - adjust based on your gripper's range

        # Current target position for the gripper
        self.current_target_position = self.open_position
        self.last_toggle_time = self.get_clock().now().nanoseconds / 1e9 # Get current time in seconds

    def publish_gripper_state(self):
        # Create a JointState message
        joint_state_msg = JointState()

        # Set the header timestamp
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        # Assign the joint names
        joint_state_msg.name = self.gripper_joint_names

        # Assign the joint positions
        # For a single joint, this will be a list with one element
        joint_state_msg.position = [self.current_target_position] * len(self.gripper_joint_names)
        # If you have multiple joints that move differently, you'd set them individually:
        # joint_state_msg.position = [left_finger_pos, right_finger_pos]

        # Publish the message
        self.joint_state_pub.publish(joint_state_msg)

        # Toggle between open and closed positions every few seconds for demonstration
        current_time = self.get_clock().now().nanoseconds / 1e9
        if (current_time - self.last_toggle_time) > 2.0: # Toggle every 2 seconds
            if self.current_target_position == self.open_position:
                self.current_target_position = self.closed_position
            else:
                self.current_target_position = self.open_position
            self.last_toggle_time = current_time

    def call_move_joint(self, pos, vel=20.0, acc=20.0):
        req = MoveJoint.Request()
        req.pos = pos
        req.vel = vel
        req.acc = acc
        req.time = 0.0
        req.radius = 0.0
        req.mode = 0
        future = self.cli_movej.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def call_move_line(self, pos, vel=[200.0, 30.0], acc=[200.0, 60.0]):
        req = MoveLine.Request()
        req.pos = pos
        req.vel = vel
        req.acc = acc
        req.time = 0.0
        req.radius = 0.0
        req.ref = 1  # DR_BASE
        req.mode = 0  # DR_MV_MOD_ABS
        req.blend_type = 0
        req.sync_type = 0
        future = self.cli_movel.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def joint_state_callback(self, msg):
        if msg.name and msg.position:
            self.current_position = msg.position[0]

    def acknowledge_gripper(self):
        request = Trigger.Request()
        future = self.ack_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Acknowledge response: {future.result().message}")
        else:
            self.get_logger().error("Failed to acknowledge gripper")

    def move_gripper(self, position):
        request = MoveToAbsolutePosition.Request()
        request.position = position
        request.velocity = 0.02  # meters per second
        request.use_gpe = False
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Trying to move to {position}')
            print(f"Success: {future.result().success}")
            print(f"Message: {future.result().message}")
        else:
            self.get_logger().error('Service call failed.')


    def execute(self):

        # ---move robot---
        self.call_move_joint([-90.0, -40.0, -50.0, 0.0, 0.0, 0.0])
        self.call_move_line([200.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # ---move gripper---
        while self.current_position is None:
            self.get_logger().info('Waiting for position data...')
            rclpy.spin_once(self)
            time.sleep(0.5)
        
        self.get_logger().info(f'Current gripper position: {self.current_position:.6f}')

        start = self.current_position
        close = min(0.08, start + 0.03)
        open_ = max(0.0, start)

        self.acknowledge_gripper()
        self.move_gripper(close)
        time.sleep(0.1)
        self.acknowledge_gripper()
        self.move_gripper(open_)   

def main(args=None):
    rclpy.init(args=args)
    node = ApplicationDemo()
    node.execute()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
