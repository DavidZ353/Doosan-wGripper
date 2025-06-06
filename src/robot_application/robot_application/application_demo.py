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
