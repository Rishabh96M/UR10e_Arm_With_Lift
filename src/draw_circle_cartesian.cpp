#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("draw_circle_cartesian");

  if (argc < 5)
  {
    RCLCPP_ERROR(node->get_logger(), "Usage: draw_circle_cartesian <x> <y> <z> <radius>");
    return 1;
  }

  double center_x = std::stod(argv[1]);
  double center_y = std::stod(argv[2]);
  double center_z = std::stod(argv[3]);
  double radius = std::stod(argv[4]);

  RCLCPP_INFO(node->get_logger(), "Center: (%.2f, %.2f, %.2f), Radius: %.2f", center_x, center_y, center_z, radius);

  // Create MoveGroupInterface
  static const std::string PLANNING_GROUP = "ur_manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  move_group.setPlanningTime(10.0);
  move_group.setMaxVelocityScalingFactor(0.1);
  move_group.setMaxAccelerationScalingFactor(0.1);

  // Get current pose as starting orientation
  geometry_msgs::msg::PoseStamped start_pose = move_group.getCurrentPose();
  tf2::Quaternion q_start;
  tf2::fromMsg(start_pose.pose.orientation, q_start);

  // Generate waypoints in a circle (assume wall at Y = 0)
  std::vector<geometry_msgs::msg::Pose> waypoints;
  int num_points = 36; // e.g., 10 degrees each

  for (int i = 0; i <= num_points; ++i)
  {
    double theta = 2 * M_PI * i / num_points;
    geometry_msgs::msg::Pose target_pose;

    target_pose.position.x = center_x + radius * cos(theta);
    target_pose.position.y = center_y; // Y fixed (wall)
    target_pose.position.z = center_z + radius * sin(theta);

    // Use current orientation (can customize)
    target_pose.orientation = start_pose.pose.orientation;

    // Check reachability using setPoseTarget + plan
    move_group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      waypoints.push_back(target_pose);
      RCLCPP_INFO(node->get_logger(), "Added reachable point (%.2f, %.2f, %.2f)", 
                  target_pose.position.x, target_pose.position.y, target_pose.position.z);
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "Skipping unreachable waypoint at (%.2f, %.2f, %.2f)", 
                  target_pose.position.x, target_pose.position.y, target_pose.position.z);
    }
  }

  if (waypoints.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "No reachable waypoints. Aborting.");
    rclcpp::shutdown();
    return 1;
  }

  // Compute Cartesian path
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  if (fraction > 0.0)
  {
    RCLCPP_INFO(node->get_logger(), "Cartesian path computed successfully (%.2f%% achieved)", fraction * 100.0);
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory;

    move_group.execute(cartesian_plan);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to compute Cartesian path.");
  }

  rclcpp::shutdown();
  return 0;
}
