import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, WorkspaceParameters
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration

from moveit.planning import MoveItPy

class MoveGroupClient(Node):
    def __init__(self):
        super().__init__('move_group_action_client')
        self._client = ActionClient(self, MoveGroup, '/move_action')

    def send_named_target(self, group_name: str, target_name: str):
        self.get_logger().info(f'Sending goal to MoveIt: group={group_name}, target={target_name}')
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = group_name
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5

        # Named target as constraint
        goal_msg.request.goal_constraints = [
            Constraints(name=target_name)
        ]

        self._client.wait_for_server()

        future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        if not future.result().accepted:
            self.get_logger().error('Goal was rejected')
            return

        goal_handle = future.result()
        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info('Motion succeeded!')
        else:
            self.get_logger().error(f'Motion failed with error code: {result.error_code.val}')

    def send_cartesian_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._client.wait_for_server()

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'manipulator'  # Change this to your actual group name
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5

        # === Create PoseStamped Target ===
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'  # Replace with your robot’s base frame
        pose.pose.position.x = 0.2
        pose.pose.position.y = 0.1
        pose.pose.position.z = 0.1
        pose.pose.orientation.w = 1.0  # No rotation

        # === Position Constraint ===
        position_constraint = PositionConstraint()
        position_constraint.header = pose.header
        position_constraint.link_name = 'gripper'  # Replace with your robot's end-effector link
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0
        position_constraint.weight = 1.0

        # Region: Box with tiny volume around the target point
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.05, 0.05, 0.05]  # Tight constraint

        position_constraint.constraint_region.primitives.append(primitive)
        position_constraint.constraint_region.primitive_poses.append(pose.pose)

        # === Orientation Constraint ===
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = pose.header
        orientation_constraint.link_name = 'gripper'  # Replace as needed
        orientation_constraint.orientation = pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0

        # === Attach constraints to request ===
        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

        goal_msg.request.goal_constraints = [constraints]

        self.get_logger().info('Sending Cartesian goal to MoveIt...')
        send_goal_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.result.error_code.val == result.result.error_code.SUCCESS:
            self.get_logger().info('Motion succeeded!')
        else:
            self.get_logger().error(f'Motion failed with error code: {result.result.error_code.val}')

        def plan_and_execute(
            robot,
            planning_component,
            logger,
            single_plan_parameters=None,
            multi_plan_parameters=None,
            ):
            """A helper function to plan and execute a motion."""
            # plan to goal
            logger.info("Planning trajectory")
            if multi_plan_parameters is not None:
                    plan_result = planning_component.plan(
                            multi_plan_parameters=multi_plan_parameters
                    )
            elif single_plan_parameters is not None:
                    plan_result = planning_component.plan(
                            single_plan_parameters=single_plan_parameters
                    )
            else:
                    plan_result = planning_component.plan()

            # execute the plan
            if plan_result:
                    logger.info("Executing plan")
                    robot_trajectory = plan_result.trajectory
                    robot.execute(robot_trajectory, controllers=[])
            else:
                    logger.error("Planning failed")



def main():
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.pose_goal")
    # instantiate MoveItPy instance and get planning component
    panda = MoveItPy(node_name="moveit_py")
    panda_arm = panda.get_planning_component("panda_arm")
    logger.info("MoveItPy instance created")

    client = MoveGroupClient()
    client.send_cartesian_goal()
    #client.send_named_target(group_name='manipulator', target_name='ready')  # Update these names as needed
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
