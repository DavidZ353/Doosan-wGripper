import rclpy
from rclpy.node import Node

from dsr_msgs2.srv import (
    MoveJoint,
    MoveLine,
    MoveSpiral
)

class DoosanMotionClient(Node):
    def __init__(self):
        super().__init__('doosan_motion_client')

        self.cli_movej = self.create_client(MoveJoint, '/motion/move_joint')
        self.cli_movel = self.create_client(MoveLine, '/motion/move_line')
        self.cli_spiral = self.create_client(MoveSpiral, '/motion/move_spiral')

        # wait for services to be available
        self.get_logger().info('Waiting for motion services...')
        self.cli_movej.wait_for_service()
        self.cli_movel.wait_for_service()
        self.cli_spiral.wait_for_service()
        self.get_logger().info('All motion services are available.')

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

    def call_move_spiral(self):
        req = MoveSpiral.Request()
        req.rev = 3
        req.rmax = 100
        req.lmax = 50
        req.v = [200.0, 60.0]
        req.a = [300.0, 100.0]
        req.time = 0.0
        req.axis = 1  # DR_AXIS_Z
        req.ref = 1   # DR_TOOL
        future = self.cli_spiral.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    node = DoosanMotionClient()

    node.get_logger().info('Sending move_joint...')
    node.call_move_joint([-90.0, -40.0, -50.0, 0.0, 0.0, 0.0])

    while rclpy.ok():
      node.get_logger().info('Sending move_line...')
      node.call_move_line([200.0, 0.0, 0.0, 0.0, 0.0, 0.0])

      node.get_logger().info('Sending move_line...')
      node.call_move_line([-200.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    #node.get_logger().info('Sending move_spiral...')
    #node.call_move_spiral()

    #node.get_logger().info('Motion sequence complete.')
    #node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
