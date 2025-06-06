import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from schunk_gripper_interfaces.srv import MoveToAbsolutePosition, Grip, Release
from std_srvs.srv import Trigger
import time

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        self.current_position = None

        # Create service client for move command
        self.move_client = self.create_client(MoveToAbsolutePosition, '/schunk/driver/EGU_60_M_B_1/move_to_absolute_position')  
        while not self.move_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for move service to become available...')

        # Create service client for acknowledge
        self.ack_client = self.create_client(Trigger, '/schunk/driver/EGU_60_M_B_1/acknowledge')
        while not self.ack_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for acknowledge service...')

        # Create service client for grip command
        self.grip_client = self.create_client(Grip, '/schunk/driver/EGU_60_M_B_1/grip')  
        while not self.grip_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for grip service to become available...')

        # Create service client for release command
        self.release_client = self.create_client(Release, '/schunk/driver/EGU_60_M_B_1/release')  
        while not self.release_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for release service to become available...')

        self.subscription = self.create_subscription(
            JointState,
            '/schunk/driver/EGU_60_M_B_1/joint_states',
            self.joint_state_callback,
            10
        )

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
        future = self.move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Trying to move to {position}')
            print(f"Success: {future.result().success}")
            print(f"Message: {future.result().message}")
        else:
            self.get_logger().error('Service call failed.')

    def grip_gripper(self, force):
        request = Grip.Request()
        request.force = force
        request.outward = False 
        request.use_gpe = False
        future = self.grip_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Trying to grip')
            print(f"Success: {future.result().success}")
            print(f"Message: {future.result().message}")
        else:
            self.get_logger().error('Service call failed.')     

    def release_gripper(self):
        request = Release.Request() 
        request.use_gpe = False
        future = self.release_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Trying to release')
            print(f"Success: {future.result().success}")
            print(f"Message: {future.result().message}")
        else:
            self.get_logger().error('Service call failed.')  

    def execute(self):

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
        time.sleep(0.5)
        self.acknowledge_gripper()
        self.move_gripper(open_)

        time.sleep(0.5)
        self.acknowledge_gripper()
        self.move_gripper(0.02)
        self.acknowledge_gripper()
        self.grip_gripper(50)
        time.sleep(1)
        self.acknowledge_gripper()
        self.release_gripper()

def main():
    rclpy.init()
    controller = GripperController()
    controller.execute()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
