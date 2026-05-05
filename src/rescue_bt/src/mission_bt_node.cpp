// mission_bt_node.cpp
// This is the main entry point for the rescue mission.
// It sets up a ROS2 node, waits for Nav2 to be ready, loads the behavior tree
// from the XML file, and then ticks it in a loop until the mission finishes.

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"  // to find installed package files
#include "behaviortree_cpp/bt_factory.h"                    // BT.CPP factory for creating trees
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "rescue_bt/bt_nodes.hpp"  // our custom BT node definitions

int main(int argc, char ** argv)
{
  // start ROS2 and create our node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mission_bt_node");

  // before we do anything, wait for Nav2's navigate_to_pose action server
  // to come online. if we try to send goals before Nav2 is ready they'll fail.
  auto nav_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    node, "navigate_to_pose");

  RCLCPP_INFO(node->get_logger(), "Waiting for Nav2 navigate_to_pose action server...");
  while (rclcpp::ok() && !nav_client->wait_for_action_server(std::chrono::seconds(2))) {
    // keep spinning so we process callbacks while waiting
    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
      "Still waiting for navigate_to_pose action server...");
    rclcpp::spin_some(node);
  }
  if (!rclcpp::ok()) {
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Nav2 action server is available!");

  // even after the action server appears, Nav2's lifecycle nodes arent
  // fully active yet. we wait 5 more seconds (50 iterations at 10Hz)
  // to let everything finish activating before sending goals.
  RCLCPP_INFO(node->get_logger(), "Waiting 5s for Nav2 lifecycle activation...");
  rclcpp::Rate wait_rate(10.0);
  for (int i = 0; i < 50 && rclcpp::ok(); ++i) {
    rclcpp::spin_some(node);
    wait_rate.sleep();
  }

  // register all our custom BT node types with the factory.
  // the string names here must match what we use in the XML tree file.
  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<rescue_bt::NavigateToWaypoint>("NavigateToWaypoint");
  factory.registerNodeType<rescue_bt::WaitSeconds>("WaitSeconds");
  factory.registerNodeType<rescue_bt::PublishObjectTF>("PublishObjectTF");
  factory.registerNodeType<rescue_bt::SetCameraAngle>("SetCameraAngle");
  factory.registerNodeType<rescue_bt::PublishDockStatus>("PublishDockStatus");
  factory.registerNodeType<rescue_bt::CheckBatteryOK>("CheckBatteryOK");
  factory.registerNodeType<rescue_bt::CheckFireSafe>("CheckFireSafe");
  factory.registerNodeType<rescue_bt::WaitForBatteryOK>("WaitForBatteryOK");

  // find the XML tree file in the installed package share directory
  std::string pkg_dir = ament_index_cpp::get_package_share_directory("rescue_bt");
  std::string tree_path = pkg_dir + "/behavior_trees/rescue_mission.xml";

  RCLCPP_INFO(node->get_logger(), "Loading behavior tree: %s", tree_path.c_str());

  // the blackboard is how BT nodes share data.
  // we put our ROS node on it so every BT node can grab it in ensureInit()
  // and use it to create publishers, subscribers, action clients, etc.
  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", node);

  // parse the XML and build the tree structure
  auto tree = factory.createTreeFromFile(tree_path, blackboard);

  RCLCPP_INFO(node->get_logger(), "Behavior tree loaded. Starting mission...");

  // main loop: tick the tree at roughly 10Hz until the mission finishes.
  // each tick progresses the tree by one step (checks conditions, advances actions).
  rclcpp::Rate rate(10.0);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    // spin_some processes any pending ROS callbacks (subscription messages,
    // action results, etc.) so our BT nodes have up-to-date data
    rclcpp::spin_some(node);

    // tick the tree once, advancing the mission by one step
    status = tree.tickOnce();
    rate.sleep();
  }

  // mission is done (either SUCCESS or FAILURE)
  RCLCPP_INFO(node->get_logger(), "Mission finished with status: %s",
    BT::toStr(status).c_str());

  rclcpp::shutdown();
  return 0;
}
