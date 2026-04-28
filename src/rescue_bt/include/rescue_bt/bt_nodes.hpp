#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <string>

namespace rescue_bt {

// ── NavigateToWaypoint (StatefulActionNode) ──
// Sends NavigateToPose goal to Nav2, returns RUNNING until complete.
class NavigateToWaypoint : public BT::StatefulActionNode {
public:
  NavigateToWaypoint(const std::string &name, const BT::NodeConfig &cfg);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using NavAction = nav2_msgs::action::NavigateToPose;
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavAction>::SharedPtr client_;
  rclcpp_action::ClientGoalHandle<NavAction>::SharedPtr goal_handle_;
  std::atomic<bool> done_{false};
  std::atomic<bool> success_{false};
};

// ── WaitSeconds (StatefulActionNode) ──
class WaitSeconds : public BT::StatefulActionNode {
public:
  WaitSeconds(const std::string &name, const BT::NodeConfig &cfg);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time end_time_;
};

// ── PublishObjectTF (SyncActionNode) ──
class PublishObjectTF : public BT::SyncActionNode {
public:
  PublishObjectTF(const std::string &name, const BT::NodeConfig &cfg);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

// ── SetCameraAngle (SyncActionNode) ──
class SetCameraAngle : public BT::SyncActionNode {
public:
  SetCameraAngle(const std::string &name, const BT::NodeConfig &cfg);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
};

// ── CheckBatteryLow (ConditionNode) ──
// Returns FAILURE when battery is low, SUCCESS when OK.
class CheckBatteryOK : public BT::ConditionNode {
public:
  CheckBatteryOK(const std::string &name, const BT::NodeConfig &cfg);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  std::atomic<bool> battery_low_{false};
};

// ── WaitForBatteryOK (StatefulActionNode) ──
class WaitForBatteryOK : public BT::StatefulActionNode {
public:
  WaitForBatteryOK(const std::string &name, const BT::NodeConfig &cfg);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  std::atomic<bool> battery_low_{true};
};

// ── CheckFireSafe (ConditionNode) ──
// Returns SUCCESS if robot is > safe_distance from fire, FAILURE otherwise.
class CheckFireSafe : public BT::ConditionNode {
public:
  CheckFireSafe(const std::string &name, const BT::NodeConfig &cfg);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace rescue_bt
