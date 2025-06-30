#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("go_to_point");

  if (argc < 4) {
    RCLCPP_ERROR(node->get_logger(), "Usage: ros2 run <pkg> go_to_point x y z [qx qy qz qw]");
    return 1;
  }

  // Parse position
  double x = std::stod(argv[1]);
  double y = std::stod(argv[2]);
  double z = std::stod(argv[3]);

  tf2::Quaternion q;
  bool has_orientation = false;

  if (argc == 8) {
    // Parse quaternion
    double qx = std::stod(argv[4]);
    double qy = std::stod(argv[5]);
    double qz = std::stod(argv[6]);
    double qw = std::stod(argv[7]);
    q = tf2::Quaternion(qx, qy, qz, qw);
    has_orientation = true;
  }

  // Create MoveGroupInterface
  static const std::string PLANNING_GROUP = "ur_manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(10);

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;

  if (has_orientation) {
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();
    move_group.setPoseTarget(target_pose);
  } else {
    // If no orientation given, disable orientation constraints by using position-only goal
    target_pose.orientation.w = 1.0; // Identity quaternion (safe fallback)
    move_group.setPoseTarget(target_pose);
  }

  move_group.setStartStateToCurrentState();

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    RCLCPP_INFO(node->get_logger(), "Plan successful. Executing...");
    move_group.execute(plan);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Planning failed.");
  }

  rclcpp::shutdown();
  return 0;
}
