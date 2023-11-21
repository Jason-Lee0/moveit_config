#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("moveit_path", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "ARM";

  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);

  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

  // Get Current State
  moveit::core::RobotStatePtr current_state_arm =
      move_group_arm.getCurrentState(10);

  std::vector<double> joint_group_positions_arm;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);

  move_group_arm.setStartStateToCurrentState();

  // Go Home
  RCLCPP_INFO(LOGGER, "Going Home");

  // joint_group_positions_arm[0] = 0.00;  // Shoulder Pan
  joint_group_positions_arm[1] = 0; // Shoulder Lift
  joint_group_positions_arm[2] = 1.50;  // Elbow
  joint_group_positions_arm[3] = 0; // Wrist 1
  //joint_group_positions_arm[4] = -1.55; // Wrist 2
  // joint_group_positions_arm[5] = 0.00;  // Wrist 3

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                      moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);
  /*while (rclcpp::ok()){
  geometry_msgs::msg::PoseStamped pose = move_group_arm.getCurrentPose();
  std::cout << "orientation.x = " << pose.pose.orientation.x << std::endl;
  std::cout << "orientation.y = " << pose.pose.orientation.y << std::endl;
  std::cout << "orientation.z = " << pose.pose.orientation.z << std::endl;
  std::cout << "orientation.w = " << pose.pose.orientation.w << std::endl;
  std::cout << "position.x  = " << pose.pose.position.x  << std::endl;
  std::cout << "position.y = " << pose.pose.position.y << std::endl;
  std::cout << "position.z = " << pose.pose.position.z << std::endl;

  std::cout << "next_point "  << std::endl;
  std::cout << "\n"  << std::endl;
  }*/

  


  // Pregrasp
  RCLCPP_INFO(LOGGER, "Pregrasp Position");
  //geometry_msgs::msg::PoseStamped target_pose1 = move_group_arm.getCurrentPose();
  geometry_msgs::msg::Pose target_pose1;

  

  target_pose1.orientation.x = 0.0146966;
  target_pose1.orientation.y = -0.583351;
  target_pose1.orientation.z = 0.81151;
  target_pose1.orientation.w = -0.0306022;
  target_pose1.position.x = 0.0267303;
  target_pose1.position.y = -0.424537;
  target_pose1.position.z = 0.661205;
  move_group_arm.setPoseTarget(target_pose1);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  // Approach
  RCLCPP_INFO(LOGGER, "Approach to object!");

  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  target_pose1.position.z -= 0.07;
  //target_pose1.position.y += 0.05;
  approach_waypoints.push_back(target_pose1);

  target_pose1.position.z -= 0.05;
  approach_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach);

  move_group_arm.execute(trajectory_approach);

  // Retreat

  RCLCPP_INFO(LOGGER, "Retreat from object!");

  /*std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
  target_pose1.position.z += 0.07;
  retreat_waypoints.push_back(target_pose1);

  target_pose1.position.z += 0.05;
  retreat_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_retreat;

  fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

  move_group_arm.execute(trajectory_retreat);*/

  rclcpp::shutdown();
  return 0;
}