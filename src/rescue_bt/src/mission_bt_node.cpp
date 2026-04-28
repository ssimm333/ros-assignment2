#include "rescue_bt/bt_nodes.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <filesystem>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mission_bt_node");
  node->declare_parameter("use_sim_time", true);

  // Register all custom BT nodes
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<rescue_bt::NavigateToWaypoint>("NavigateToWaypoint");
  factory.registerNodeType<rescue_bt::WaitSeconds>("WaitSeconds");
  factory.registerNodeType<rescue_bt::PublishObjectTF>("PublishObjectTF");
  factory.registerNodeType<rescue_bt::SetCameraAngle>("SetCameraAngle");
  factory.registerNodeType<rescue_bt::CheckBatteryOK>("CheckBatteryOK");
  factory.registerNodeType<rescue_bt::WaitForBatteryOK>("WaitForBatteryOK");
  factory.registerNodeType<rescue_bt::CheckFireSafe>("CheckFireSafe");

  // Load BT XML
  std::string pkg_dir =
      ament_index_cpp::get_package_share_directory("rescue_bt");
  std::string bt_xml = pkg_dir + "/behavior_trees/rescue_mission.xml";

  if (!std::filesystem::exists(bt_xml)) {
    RCLCPP_FATAL(node->get_logger(), "BT XML not found: %s", bt_xml.c_str());
    return 1;
  }

  // Set up blackboard with ROS node handle
  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", node);

  auto tree = factory.createTreeFromFile(bt_xml, blackboard);

  // Console logger for debugging
  BT::StdCoutLogger logger(tree);

  RCLCPP_INFO(node->get_logger(), "=== Mission BT loaded, starting execution ===");

  // Tick loop at 10 Hz
  rclcpp::Rate rate(10);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    rclcpp::spin_some(node);
    status = tree.tickOnce();
    rate.sleep();
  }

  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "=== MISSION COMPLETE ===");
  } else {
    RCLCPP_ERROR(node->get_logger(), "=== MISSION FAILED ===");
  }

  rclcpp::shutdown();
  return 0;
}
