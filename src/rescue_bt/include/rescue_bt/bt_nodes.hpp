#ifndef RESCUE_BT__BT_NODES_HPP_
#define RESCUE_BT__BT_NODES_HPP_

#include <string>
#include <chrono>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"

namespace rescue_bt
{

// ─── Async Action: Navigate to a waypoint via Nav2 ───────────────

class NavigateToWaypoint : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigateToWaypoint(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void ensureInit();
  BT::NodeStatus sendGoal(double x, double y, double theta, const std::string & desc);
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  GoalHandle::SharedPtr goal_handle_;
  bool goal_done_;
  bool goal_succeeded_;
  bool goal_sent_;
};

// ─── Async Action: Wait for a specified duration ─────────────────

class WaitSeconds : public BT::StatefulActionNode
{
public:
  WaitSeconds(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::chrono::steady_clock::time_point end_time_;
};

// ─── Sync Action: Publish a static TF for a named object ────────

class PublishObjectTF : public BT::SyncActionNode
{
public:
  PublishObjectTF(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void ensureInit();
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

// ─── Sync Action: Set camera joint angle ─────────────────────────

class SetCameraAngle : public BT::SyncActionNode
{
public:
  SetCameraAngle(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void ensureInit();
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
};

// ─── Sync Action: Publish docking status ─────────────────────────

class PublishDockStatus : public BT::SyncActionNode
{
public:
  PublishDockStatus(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void ensureInit();
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
};

// ─── Condition: Check battery is OK ──────────────────────────────

class CheckBatteryOK : public BT::ConditionNode
{
public:
  CheckBatteryOK(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void ensureInit();
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  bool battery_low_;
};

// ─── Condition: Check distance to fire is safe ───────────────────

class CheckFireSafe : public BT::ConditionNode
{
public:
  CheckFireSafe(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void ensureInit();
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

// ─── Async Action: Wait until battery is recharged ───────────────

class WaitForBatteryOK : public BT::StatefulActionNode
{
public:
  WaitForBatteryOK(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void ensureInit();
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  bool battery_low_;
};

}  // namespace rescue_bt

#endif  // RESCUE_BT__BT_NODES_HPP_
