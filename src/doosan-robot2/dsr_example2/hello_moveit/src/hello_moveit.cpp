#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/mesh_triangle.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometric_shapes/shape_operations.h>   // Für Mesh-Ladefunktion
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <thread> 


int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "manipulator");

  auto jmg = move_group_interface.getRobotModel()->getJointModelGroup("manipulator");

  if (!jmg)
  {
    RCLCPP_ERROR(logger, "JointModelGroup 'manipulator' not found!");
  } else {
    RCLCPP_INFO(logger, "LinkModelNames in JointModelGroup 'manipulator':");
    for (const auto& link_name : jmg->getLinkModelNames())
    {
      RCLCPP_INFO(logger, "  %s", link_name.c_str());
    }
  }

  RCLCPP_INFO(logger, "Planning frame_move: %s", move_group_interface.getPlanningFrame().c_str());
  

  // Add the stacker object
  moveit_msgs::msg::CollisionObject collision_stacker;
  collision_stacker.header.frame_id = move_group_interface.getPlanningFrame();
  collision_stacker.id = "stacker";

  const std::string stacker_mesh_path = "package://dsr_description2/meshes/container_corner/Depot_Stacker.dae";
  shapes::Mesh* stacker_mesh = shapes::createMeshFromResource(stacker_mesh_path, Eigen::Vector3d(0.001, 0.001, 0.001));

  if (!stacker_mesh) {
    RCLCPP_ERROR(logger, "Failed to load mesh from %s", stacker_mesh_path.c_str());
  } else {
    shape_msgs::msg::Mesh stacker_mesh_msg;
    shapes::ShapeMsg tmp_msg;
    shapes::constructMsgFromShape(stacker_mesh, tmp_msg);
    stacker_mesh_msg = boost::get<shape_msgs::msg::Mesh>(tmp_msg);

    collision_stacker.meshes.push_back(stacker_mesh_msg);

    geometry_msgs::msg::Pose stacker_pose;
    stacker_pose.orientation.x = 0.5;
    stacker_pose.orientation.y = 0.5;
    stacker_pose.orientation.z = 0.5;
    stacker_pose.orientation.w = 0.5;
    stacker_pose.position.x = 0.35;
    stacker_pose.position.y = 0.9;
    stacker_pose.position.z = 0.48;
    collision_stacker.mesh_poses.push_back(stacker_pose);

    collision_stacker.operation = collision_stacker.ADD;

    delete stacker_mesh;
  }

  // Add first container corner
  moveit_msgs::msg::CollisionObject collision_corner_1;
  collision_corner_1.header.frame_id = move_group_interface.getPlanningFrame();
  collision_corner_1.id = "corner1";

  const std::string corner_mesh_path = "package://dsr_description2/meshes/container_corner/Containerecken_oben_rechts.dae";
  shapes::Mesh* corner_mesh_1 = shapes::createMeshFromResource(corner_mesh_path, Eigen::Vector3d(0.001, 0.001, 0.001));

  if (!corner_mesh_1) {
    RCLCPP_ERROR(logger, "Failed to load mesh from %s", corner_mesh_path.c_str());
  } else {
    shape_msgs::msg::Mesh corner_mesh_msg_1;
    shapes::ShapeMsg tmp_msg;
    shapes::constructMsgFromShape(corner_mesh_1, tmp_msg);
    corner_mesh_msg_1 = boost::get<shape_msgs::msg::Mesh>(tmp_msg);

    collision_corner_1.meshes.push_back(corner_mesh_msg_1);

    geometry_msgs::msg::Pose corner_pose_1;
    corner_pose_1.orientation.w = 1.0;
    corner_pose_1.position.x = 0.5;
    corner_pose_1.position.y = 1.0;
    corner_pose_1.position.z = 0.4;
    collision_corner_1.mesh_poses.push_back(corner_pose_1);

    collision_corner_1.operation = collision_corner_1.ADD;

    delete corner_mesh_1;
  }

  // Add second container corner
  moveit_msgs::msg::CollisionObject collision_corner_2;
  collision_corner_2.header.frame_id = move_group_interface.getPlanningFrame();
  collision_corner_2.id = "corner2";  

  shapes::Mesh* corner_mesh_2 = shapes::createMeshFromResource(corner_mesh_path, Eigen::Vector3d(0.001, 0.001, 0.001));

  if (!corner_mesh_2) {
    RCLCPP_ERROR(logger, "Failed to load mesh from %s", corner_mesh_path.c_str());
  } else {
    shape_msgs::msg::Mesh corner_mesh_msg_2;
    shapes::ShapeMsg tmp_msg;
    shapes::constructMsgFromShape(corner_mesh_2, tmp_msg);
    corner_mesh_msg_2 = boost::get<shape_msgs::msg::Mesh>(tmp_msg);

    collision_corner_2.meshes.push_back(corner_mesh_msg_2);

    geometry_msgs::msg::Pose corner_pose_2;
    corner_pose_2.orientation.x = 0.0;
    corner_pose_2.orientation.y = -1.0;
    corner_pose_2.orientation.z = 0.0;
    corner_pose_2.orientation.w = 0.0;
    corner_pose_2.position.x = -0.10;
    corner_pose_2.position.y = 0.20;
    corner_pose_2.position.z = 0.80;
    collision_corner_2.mesh_poses.push_back(corner_pose_2);

    collision_corner_2.operation = collision_corner_2.ADD;

    delete corner_mesh_2;
  }

  // Planning scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::vector<moveit_msgs::msg::CollisionObject> objs = {collision_stacker, collision_corner_1, collision_corner_2};
  planning_scene_interface.applyCollisionObjects(objs);
  rclcpp::sleep_for(std::chrono::milliseconds(100));


  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create a closures for visualization
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 2.05;
      msg.translation().y() = 0.8;
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XXXLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };
  
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools,
      jmg = move_group_interface.getRobotModel()->getJointModelGroup("gripper"),
      ee_link = move_group_interface.getRobotModel()->getLinkModel("left_finger")]
      (auto const& trajectory) {
        moveit_visual_tools.publishTrajectoryLine(trajectory, ee_link, jmg);
      };


  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 0.707156;
    msg.orientation.y = -0.707058;
    msg.orientation.z = -8.56808e-06;
    msg.orientation.w = 7.56471e-06;
    msg.position.x = 0.34998;
    msg.position.y = 0.89995;
    msg.position.z = 0.67592;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  std::string planning_frame = move_group_interface.getPlanningFrame();
  RCLCPP_INFO(logger, "Planning frame: %s", planning_frame.c_str());


  // Create a plan to that target pose
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  moveit_visual_tools.trigger();
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success) {
    draw_trajectory_tool_path(plan.trajectory_);
    moveit_visual_tools.trigger();
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    draw_title("Executing");
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan);
  } else {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
  spinner.join();  // <--- Join the thread before exiting
  return 0;
}