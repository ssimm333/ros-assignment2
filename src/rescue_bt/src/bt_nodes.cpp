#include "rescue_bt/bt_nodes.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

namespace rescue_bt {

// ═══════════════════════════════════════════════
//  NavigateToWaypoint
// ═══════════════════════════════════════════════

NavigateToWaypoint::NavigateToWaypoint(const std::string &name,
                                       const BT::NodeConfig &cfg)
    : BT::StatefulActionNode(name, cfg) {
  node_ = cfg.blackboard->get<rclcpp::Node::SharedPtr>("node");
  client_ = rclcpp_action::create_client<NavAction>(node_, "navigate_to_pose");
}

BT::PortsList NavigateToWaypoint::providedPorts() {
  return {
      BT::InputPort<double>("x"),
      BT::InputPort<double>("y"),
      BT::InputPort<double>("theta", 0.0, "Goal yaw"),
      BT::InputPort<std::string>("description", "", "Waypoint label"),
  };
}

BT::NodeStatus NavigateToWaypoint::onStart() {
  double x, y, theta;
  getInput("x", x);
  getInput("y", y);
  getInput("theta", theta);
  std::string desc;
  getInput("description", desc);

  if (!client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(node_->get_logger(), "Nav2 action server not available");
    return BT::NodeStatus::FAILURE;
  }

  auto goal = NavAction::Goal();
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = node_->now();
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.orientation.z = std::sin(theta / 2.0);
  goal.pose.pose.orientation.w = std::cos(theta / 2.0);

  done_ = false;
  success_ = false;

  RCLCPP_INFO(node_->get_logger(), "Navigating to (%.1f, %.1f) [%s]", x, y,
              desc.c_str());

  auto opts = rclcpp_action::Client<NavAction>::SendGoalOptions();
  opts.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<NavAction>::WrappedResult
                 &result) {
        done_ = true;
        success_ =
            (result.code == rclcpp_action::ResultCode::SUCCEEDED);
      };

  client_->async_send_goal(goal, opts);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToWaypoint::onRunning() {
  if (done_) {
    return success_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void NavigateToWaypoint::onHalted() {
  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
  }
  RCLCPP_INFO(node_->get_logger(), "Navigation halted");
}

// ═══════════════════════════════════════════════
//  WaitSeconds
// ═══════════════════════════════════════════════

WaitSeconds::WaitSeconds(const std::string &name, const BT::NodeConfig &cfg)
    : BT::StatefulActionNode(name, cfg) {
  node_ = cfg.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList WaitSeconds::providedPorts() {
  return {BT::InputPort<double>("duration", 1.0, "Seconds to wait")};
}

BT::NodeStatus WaitSeconds::onStart() {
  double dur;
  getInput("duration", dur);
  end_time_ = node_->now() + rclcpp::Duration::from_seconds(dur);
  RCLCPP_INFO(node_->get_logger(), "Waiting %.1f seconds", dur);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitSeconds::onRunning() {
  if (node_->now() >= end_time_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void WaitSeconds::onHalted() {}

// ═══════════════════════════════════════════════
//  PublishObjectTF
// ═══════════════════════════════════════════════

PublishObjectTF::PublishObjectTF(const std::string &name,
                                 const BT::NodeConfig &cfg)
    : BT::SyncActionNode(name, cfg) {
  node_ = cfg.blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
}

BT::PortsList PublishObjectTF::providedPorts() {
  return {
      BT::InputPort<std::string>("object_name"),
      BT::InputPort<double>("x"),
      BT::InputPort<double>("y"),
  };
}

BT::NodeStatus PublishObjectTF::tick() {
  std::string name;
  double x, y;
  getInput("object_name", name);
  getInput("x", x);
  getInput("y", y);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = node_->now();
  tf.header.frame_id = "map";
  tf.child_frame_id = name + "_location";
  tf.transform.translation.x = x;
  tf.transform.translation.y = y;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation.w = 1.0;

  tf_broadcaster_->sendTransform(tf);
  RCLCPP_INFO(node_->get_logger(), "Published TF for '%s' at (%.1f, %.1f)",
              name.c_str(), x, y);
  return BT::NodeStatus::SUCCESS;
}

// ═══════════════════════════════════════════════
//  SetCameraAngle
// ═══════════════════════════════════════════════

SetCameraAngle::SetCameraAngle(const std::string &name,
                               const BT::NodeConfig &cfg)
    : BT::SyncActionNode(name, cfg) {
  node_ = cfg.blackboard->get<rclcpp::Node::SharedPtr>("node");
  pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/camera_controller/commands", 10);
}

BT::PortsList SetCameraAngle::providedPorts() {
  return {BT::InputPort<double>("angle", 0.0, "Camera angle in radians")};
}

BT::NodeStatus SetCameraAngle::tick() {
  double angle;
  getInput("angle", angle);

  std_msgs::msg::Float64MultiArray msg;
  msg.data.push_back(angle);
  pub_->publish(msg);

  RCLCPP_INFO(node_->get_logger(), "Camera angle set to %.3f rad (%.1f deg)",
              angle, angle * 180.0 / M_PI);
  return BT::NodeStatus::SUCCESS;
}

// ═══════════════════════════════════════════════
//  CheckBatteryOK
// ═══════════════════════════════════════════════

CheckBatteryOK::CheckBatteryOK(const std::string &name,
                               const BT::NodeConfig &cfg)
    : BT::ConditionNode(name, cfg) {
  node_ = cfg.blackboard->get<rclcpp::Node::SharedPtr>("node");
  sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "battery_level_low", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        battery_low_ = msg->data;
      });
}

BT::PortsList CheckBatteryOK::providedPorts() { return {}; }

BT::NodeStatus CheckBatteryOK::tick() {
  return battery_low_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

// ═══════════════════════════════════════════════
//  WaitForBatteryOK
// ═══════════════════════════════════════════════

WaitForBatteryOK::WaitForBatteryOK(const std::string &name,
                                   const BT::NodeConfig &cfg)
    : BT::StatefulActionNode(name, cfg) {
  node_ = cfg.blackboard->get<rclcpp::Node::SharedPtr>("node");
  sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "battery_level_low", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        battery_low_ = msg->data;
      });
}

BT::PortsList WaitForBatteryOK::providedPorts() { return {}; }

BT::NodeStatus WaitForBatteryOK::onStart() {
  RCLCPP_INFO(node_->get_logger(), "Waiting at dock for battery recharge...");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForBatteryOK::onRunning() {
  if (!battery_low_) {
    RCLCPP_INFO(node_->get_logger(), "Battery recharged!");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void WaitForBatteryOK::onHalted() {}

// ═══════════════════════════════════════════════
//  CheckFireSafe
// ═══════════════════════════════════════════════

CheckFireSafe::CheckFireSafe(const std::string &name,
                             const BT::NodeConfig &cfg)
    : BT::ConditionNode(name, cfg) {
  node_ = cfg.blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList CheckFireSafe::providedPorts() {
  return {
      BT::InputPort<double>("fire_x", -14.2, "Fire X position"),
      BT::InputPort<double>("fire_y", 10.8, "Fire Y position"),
      BT::InputPort<double>("safe_distance", 3.0, "Minimum safe distance"),
  };
}

BT::NodeStatus CheckFireSafe::tick() {
  double fire_x, fire_y, safe_dist;
  getInput("fire_x", fire_x);
  getInput("fire_y", fire_y);
  getInput("safe_distance", safe_dist);

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    // TF not available yet — assume safe
    return BT::NodeStatus::SUCCESS;
  }

  double dx = tf.transform.translation.x - fire_x;
  double dy = tf.transform.translation.y - fire_y;
  double dist = std::sqrt(dx * dx + dy * dy);

  if (dist < safe_dist) {
    RCLCPP_WARN(node_->get_logger(),
                "TOO CLOSE TO FIRE! dist=%.1f < %.1f", dist, safe_dist);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

} // namespace rescue_bt
