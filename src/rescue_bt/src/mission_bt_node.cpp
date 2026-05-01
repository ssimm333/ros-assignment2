#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "rescue_bt/bt_nodes.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mission_bt_node");

  // Wait for Nav2 to be fully active before loading the BT
  auto nav_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    node, "navigate_to_pose");

  RCLCPP_INFO(node->get_logger(), "Waiting for Nav2 navigate_to_pose action server...");
  while (rclcpp::ok() && !nav_client->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
      "Still waiting for navigate_to_pose action server...");
    rclcpp::spin_some(node);
  }
  if (!rclcpp::ok()) {
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Nav2 action server is available!");

  // Extra delay to let Nav2 lifecycle activation complete
  // (action server exists before lifecycle is fully active)
  RCLCPP_INFO(node->get_logger(), "Waiting 5s for Nav2 lifecycle activation...");
  rclcpp::Rate wait_rate(10.0);
  for (int i = 0; i < 50 && rclcpp::ok(); ++i) {
    rclcpp::spin_some(node);
    wait_rate.sleep();
  }

  // Register all BT node types
  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<rescue_bt::NavigateToWaypoint>("NavigateToWaypoint");
  factory.registerNodeType<rescue_bt::WaitSeconds>("WaitSeconds");
  factory.registerNodeType<rescue_bt::PublishObjectTF>("PublishObjectTF");
  factory.registerNodeType<rescue_bt::SetCameraAngle>("SetCameraAngle");
  factory.registerNodeType<rescue_bt::PublishDockStatus>("PublishDockStatus");
  factory.registerNodeType<rescue_bt::CheckBatteryOK>("CheckBatteryOK");
  factory.registerNodeType<rescue_bt::CheckFireSafe>("CheckFireSafe");
  factory.registerNodeType<rescue_bt::WaitForBatteryOK>("WaitForBatteryOK");

  // Load tree XML from installed share directory
  std::string pkg_dir = ament_index_cpp::get_package_share_directory("rescue_bt");
  std::string tree_path = pkg_dir + "/behavior_trees/rescue_mission.xml";

  RCLCPP_INFO(node->get_logger(), "Loading behavior tree: %s", tree_path.c_str());

  // Put ROS node on the blackboard so BT nodes can access it
  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", node);

  auto tree = factory.createTreeFromFile(tree_path, blackboard);

  RCLCPP_INFO(node->get_logger(), "Behavior tree loaded. Starting mission...");

  // Tick loop at ~10 Hz
  rclcpp::Rate rate(10.0);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    // Process ROS callbacks (subscriptions, action results, etc.)
    rclcpp::spin_some(node);

    status = tree.tickOnce();
    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "Mission finished with status: %s",
    BT::toStr(status).c_str());

  rclcpp::shutdown();
  return 0;
}
