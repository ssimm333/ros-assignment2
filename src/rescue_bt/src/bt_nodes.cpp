#include "rescue_bt/bt_nodes.hpp"

#include <cmath>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace rescue_bt
{

// ═══════════════════════════════════════════════════════════════════
// NavigateToWaypoint
// ═══════════════════════════════════════════════════════════════════

NavigateToWaypoint::NavigateToWaypoint(
  const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config),
  goal_done_(false),
  goal_succeeded_(false),
  goal_sent_(false)
{
}

void NavigateToWaypoint::ensureInit()
{
  if (!node_) {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    action_client_ = rclcpp_action::create_client<NavigateToPose>(
      node_, "navigate_to_pose");
  }
}

BT::PortsList NavigateToWaypoint::providedPorts()
{
  return {
    BT::InputPort<double>("x"),
    BT::InputPort<double>("y"),
    BT::InputPort<double>("theta", 0.0, "Goal orientation (yaw)"),
    BT::InputPort<std::string>("description", "", "Waypoint label for logging"),
  };
}

BT::NodeStatus NavigateToWaypoint::onStart()
{
  ensureInit();

  double x, y, theta;
  if (!getInput("x", x) || !getInput("y", y)) {
    RCLCPP_ERROR(node_->get_logger(), "NavigateToWaypoint: missing x or y port");
    return BT::NodeStatus::FAILURE;
  }
  getInput("theta", theta);

  std::string desc;
  getInput("description", desc);
  RCLCPP_INFO(node_->get_logger(), "Navigating to %s (%.1f, %.1f)", desc.c_str(), x, y);

  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node_->get_logger(), "navigate_to_pose action server not available");
    return BT::NodeStatus::FAILURE;
  }

  goal_done_ = false;
  goal_succeeded_ = false;
  goal_sent_ = false;

  return sendGoal(x, y, theta, desc);
}

BT::NodeStatus NavigateToWaypoint::sendGoal(
  double x, double y, double theta, const std::string & desc)
{
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = node_->now();
  goal_msg.pose.pose.position.x = x;
  goal_msg.pose.pose.position.y = y;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  goal_msg.pose.pose.orientation.x = q.x();
  goal_msg.pose.pose.orientation.y = q.y();
  goal_msg.pose.pose.orientation.z = q.z();
  goal_msg.pose.pose.orientation.w = q.w();

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
    [this](const GoalHandle::WrappedResult & result) {
      goal_done_ = true;
      goal_succeeded_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
    };

  auto future = action_client_->async_send_goal(goal_msg, send_goal_options);
  if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_WARN(node_->get_logger(), "Failed to send navigation goal, will retry...");
    return BT::NodeStatus::RUNNING;
  }

  goal_handle_ = future.get();
  if (!goal_handle_) {
    RCLCPP_WARN(node_->get_logger(), "Navigation goal rejected (Nav2 may still be activating), will retry...");
    return BT::NodeStatus::RUNNING;
  }

  goal_sent_ = true;
  RCLCPP_INFO(node_->get_logger(), "Navigation goal accepted for %s", desc.c_str());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToWaypoint::onRunning()
{
  if (!goal_sent_) {
    // Goal was rejected or failed to send — retry
    double x, y, theta;
    std::string desc;
    getInput("x", x);
    getInput("y", y);
    getInput("theta", theta);
    getInput("description", desc);
    return sendGoal(x, y, theta, desc);
  }
  if (goal_done_) {
    return goal_succeeded_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void NavigateToWaypoint::onHalted()
{
  if (goal_handle_) {
    action_client_->async_cancel_goal(goal_handle_);
    RCLCPP_INFO(node_->get_logger(), "NavigateToWaypoint: navigation cancelled");
  }
}

// ═══════════════════════════════════════════════════════════════════
// WaitSeconds
// ═══════════════════════════════════════════════════════════════════

WaitSeconds::WaitSeconds(
  const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList WaitSeconds::providedPorts()
{
  return {
    BT::InputPort<double>("seconds", 1.0, "Duration to wait"),
  };
}

BT::NodeStatus WaitSeconds::onStart()
{
  double seconds;
  getInput("seconds", seconds);
  end_time_ = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(static_cast<int>(seconds * 1000));
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitSeconds::onRunning()
{
  if (std::chrono::steady_clock::now() >= end_time_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void WaitSeconds::onHalted()
{
}

// ═══════════════════════════════════════════════════════════════════
// PublishObjectTF
// ═══════════════════════════════════════════════════════════════════

PublishObjectTF::PublishObjectTF(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

void PublishObjectTF::ensureInit()
{
  if (!node_) {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  }
}

BT::PortsList PublishObjectTF::providedPorts()
{
  return {
    BT::InputPort<std::string>("frame_name", "Name of the TF frame to publish"),
    BT::InputPort<double>("x"),
    BT::InputPort<double>("y"),
  };
}

BT::NodeStatus PublishObjectTF::tick()
{
  ensureInit();

  std::string frame_name;
  double x, y;
  if (!getInput("frame_name", frame_name) || !getInput("x", x) || !getInput("y", y)) {
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = node_->now();
  t.header.frame_id = "map";
  t.child_frame_id = frame_name;
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = 0.0;
  t.transform.rotation.w = 1.0;

  tf_broadcaster_->sendTransform(t);
  RCLCPP_INFO(node_->get_logger(), "Published TF: %s at (%.1f, %.1f)", frame_name.c_str(), x, y);
  return BT::NodeStatus::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════
// SetCameraAngle
// ═══════════════════════════════════════════════════════════════════

SetCameraAngle::SetCameraAngle(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

void SetCameraAngle::ensureInit()
{
  if (!node_) {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    pub_ = node_->create_publisher<std_msgs::msg::Float64>(
      "/camera_controller/commands", 10);
  }
}

BT::PortsList SetCameraAngle::providedPorts()
{
  return {
    BT::InputPort<double>("angle", 0.0, "Camera yaw angle in radians"),
  };
}

BT::NodeStatus SetCameraAngle::tick()
{
  ensureInit();

  double angle;
  getInput("angle", angle);

  auto msg = std_msgs::msg::Float64();
  msg.data = angle;
  pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "Camera angle set to %.4f rad", angle);
  return BT::NodeStatus::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════
// PublishDockStatus
// ═══════════════════════════════════════════════════════════════════

PublishDockStatus::PublishDockStatus(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

void PublishDockStatus::ensureInit()
{
  if (!node_) {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    pub_ = node_->create_publisher<std_msgs::msg::Bool>("/is_docked", 10);
  }
}

BT::PortsList PublishDockStatus::providedPorts()
{
  return {
    BT::InputPort<bool>("docked", "true to dock, false to undock"),
  };
}

BT::NodeStatus PublishDockStatus::tick()
{
  ensureInit();

  bool docked;
  if (!getInput("docked", docked)) {
    return BT::NodeStatus::FAILURE;
  }

  auto msg = std_msgs::msg::Bool();
  msg.data = docked;
  pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "Dock status: %s", docked ? "DOCKED" : "UNDOCKED");
  return BT::NodeStatus::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════
// CheckBatteryOK
// ═══════════════════════════════════════════════════════════════════

CheckBatteryOK::CheckBatteryOK(
  const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config),
  battery_low_(false)
{
}

void CheckBatteryOK::ensureInit()
{
  if (!node_) {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/battery_level_low", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        battery_low_ = msg->data;
      });
  }
}

BT::PortsList CheckBatteryOK::providedPorts()
{
  return {};
}

BT::NodeStatus CheckBatteryOK::tick()
{
  ensureInit();
  if (battery_low_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "CheckBatteryOK: battery is LOW — triggering dock");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════
// CheckFireSafe
// ═══════════════════════════════════════════════════════════════════

static constexpr double FIRE_X = -14.2;
static constexpr double FIRE_Y = 10.8;
static constexpr double FIRE_SAFE_DISTANCE = 3.0;

CheckFireSafe::CheckFireSafe(
  const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

void CheckFireSafe::ensureInit()
{
  if (!node_) {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }
}

BT::PortsList CheckFireSafe::providedPorts()
{
  return {};
}

BT::NodeStatus CheckFireSafe::tick()
{
  ensureInit();

  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "CheckFireSafe: cannot get robot pose: %s", ex.what());
    return BT::NodeStatus::SUCCESS;
  }

  double dx = t.transform.translation.x - FIRE_X;
  double dy = t.transform.translation.y - FIRE_Y;
  double dist = std::sqrt(dx * dx + dy * dy);

  if (dist < FIRE_SAFE_DISTANCE) {
    RCLCPP_WARN(node_->get_logger(), "TOO CLOSE TO FIRE! dist=%.2f m", dist);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════
// WaitForBatteryOK
// ═══════════════════════════════════════════════════════════════════

WaitForBatteryOK::WaitForBatteryOK(
  const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config),
  battery_low_(true)
{
}

void WaitForBatteryOK::ensureInit()
{
  if (!node_) {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/battery_level_low", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        battery_low_ = msg->data;
      });
  }
}

BT::PortsList WaitForBatteryOK::providedPorts()
{
  return {};
}

BT::NodeStatus WaitForBatteryOK::onStart()
{
  ensureInit();
  RCLCPP_INFO(node_->get_logger(), "Waiting for battery to recharge...");
  return battery_low_ ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
}

BT::NodeStatus WaitForBatteryOK::onRunning()
{
  return battery_low_ ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
}

void WaitForBatteryOK::onHalted()
{
}

}  // namespace rescue_bt
