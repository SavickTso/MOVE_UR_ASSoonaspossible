#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  node->set_parameter(rclcpp::Parameter("use_sim_time", true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // TF2 setup: create the Buffer and Listener
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // Get the current pose of the end-effector
  std::string base_frame = move_group_interface.getPlanningFrame().c_str();
  std::string tool_frame = move_group_interface.getEndEffectorLink().c_str();
  RCLCPP_INFO(logger, "Planning Frame: %s", move_group_interface.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
  // Look up the transform from base_link to tool0
  geometry_msgs::msg::TransformStamped transform_stamped;

  for (int i = 0; i < 5; ++i)
  {
    try
    {
      transform_stamped = tf_buffer->lookupTransform(base_frame, tool_frame, tf2::TimePointZero);
      RCLCPP_INFO(logger, "TF Pose: translation( x: %f, y: %f, z: %f ) rotation( x: %f, y: %f, z: %f, w: %f )",
                  transform_stamped.transform.translation.x,
                  transform_stamped.transform.translation.y,
                  transform_stamped.transform.translation.z,
                  transform_stamped.transform.rotation.x,
                  transform_stamped.transform.rotation.y,
                  transform_stamped.transform.rotation.z,
                  transform_stamped.transform.rotation.w);
      break;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(logger, "Attempt %d: Could not transform base_link to tool0: %s", i + 1, ex.what());
      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }
  }

  // Define waypoints for the Cartesian path
  std::vector<geometry_msgs::msg::Pose> waypoints;
  auto const current_pose = [&transform_stamped]{
    geometry_msgs::msg::Pose msg;
    msg.position.x = transform_stamped.transform.translation.x;
    msg.position.y = transform_stamped.transform.translation.y;
    msg.position.z = transform_stamped.transform.translation.z;
    msg.orientation.x = transform_stamped.transform.rotation.x;
    msg.orientation.y = transform_stamped.transform.rotation.y;
    msg.orientation.z = transform_stamped.transform.rotation.z;
    msg.orientation.w = transform_stamped.transform.rotation.w;
    return msg;
  }();
  // Start with the current pose
  waypoints.push_back(current_pose);
  // Move up in the Z direction
  geometry_msgs::msg::Pose target_pose = current_pose;
  target_pose.position.z += 0.1; // Move 10 cm up
  waypoints.push_back(target_pose);
  // target_pose.position.y += 0.1; // Move 10 cm along Y-axis
  // waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(logger, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
  move_group_interface.execute(trajectory); // Execute the trajectory

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;

  // *******DANGER ZONE, planned motion could be random***************
  // // Pose based command
  // auto const target_pose = [&transform_stamped]{
  //   geometry_msgs::msg::Pose msg;
  //   msg.position.z = transform_stamped.transform.translation.z + 0.1;
  //   msg.position.x = transform_stamped.transform.translation.x;
  //   msg.position.y = transform_stamped.transform.translation.y;
  //   msg.orientation.x = transform_stamped.transform.rotation.x;
  //   msg.orientation.y = transform_stamped.transform.rotation.y;
  //   msg.orientation.z = transform_stamped.transform.rotation.z;
  //   msg.orientation.w = transform_stamped.transform.rotation.w;
  //   return msg;
  // }();

  // move_group_interface.setPoseTarget(target_pose);
  // RCLCPP_INFO(logger, "Target Pose: position( x: %f, y: %f, z: %f ) orientation( x: %f, y: %f, z: %f, w: %f )",
  //             target_pose.position.x, target_pose.position.y, target_pose.position.z,
  //             target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);

  // // Create a plan to that target pose
  // auto const [success, plan] = [&move_group_interface]{
  //   moveit::planning_interface::MoveGroupInterface::Plan msg;
  //   auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  //   return std::make_pair(ok, msg);
  // }();

  // // Execute the plan
  // if(success) {
  //   move_group_interface.execute(plan);
  // } else {
  //   RCLCPP_ERROR(logger, "Planning failed!");
  // }

  // // Shutdown ROS
  // rclcpp::shutdown();
  // return 0;
}
