// bt_nodes.hpp
// Header file for all the custom behavior tree nodes used in the rescue mission.
// Each class here is a different type of BT node that gets registered with the
// BehaviorTreeFactory in mission_bt_node.cpp and then referenced by name in
// the rescue_mission.xml tree file.
//
// There are three base types used here from BT.CPP:
//   StatefulActionNode  - for actions that take multiple ticks to finish (async)
//   SyncActionNode      - for actions that complete in a single tick
//   ConditionNode       - for checks that return SUCCESS or FAILURE instantly

#ifndef RESCUE_BT__BT_NODES_HPP_
#define RESCUE_BT__BT_NODES_HPP_

#include <string>
#include <chrono>

// BT.CPP core headers for the node base classes and the factory
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

// ROS2 headers
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"               // needed for action clients (Nav2 uses actions)
#include "nav2_msgs/action/navigate_to_pose.hpp"          // the Nav2 NavigateToPose action type
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"                          // used for battery and dock status messages
#include "std_msgs/msg/float64.hpp"                       // used for the camera angle commands
#include "tf2_ros/buffer.h"                               // TF2 transform lookup buffer
#include "tf2_ros/transform_listener.h"                   // listens for transforms on /tf
#include "tf2_ros/static_transform_broadcaster.h"         // broadcasts static transforms (for object locations)

namespace rescue_bt
{

// NavigateToWaypoint
// Sends a goal to the Nav2 navigate_to_pose action server and waits for it to finish.
// This is a StatefulActionNode because navigation takes many ticks to complete,
// so onStart sends the goal, onRunning checks if its done, and onHalted cancels it.

class NavigateToWaypoint : public BT::StatefulActionNode
{
public:
  // type aliases to keep the code shorter
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigateToWaypoint(const std::string & name, const BT::NodeConfig & config);

  // providedPorts tells BT.CPP what input/output ports this node has
  // (these are the attributes you set in the XML like x="15.1" y="11.9")
  static BT::PortsList providedPorts();

  // StatefulActionNode lifecycle: onStart -> onRunning (repeated) -> onHalted (if interrupted)
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  // lazy initialization, grabs the ROS node from the blackboard on first use
  void ensureInit();

  // helper that actually sends the navigation goal to Nav2
  BT::NodeStatus sendGoal(double x, double y, double theta, const std::string & desc);

  rclcpp::Node::SharedPtr node_;                                // shared ROS node from the blackboard
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;  // action client to talk to Nav2
  GoalHandle::SharedPtr goal_handle_;                           // handle to the current goal (used for cancelling)
  bool goal_done_;       // set true by the result callback when Nav2 finishes
  bool goal_succeeded_;  // whether the navigation actually succeeded
  bool goal_sent_;       // tracks if the goal was accepted (used for retry logic)
};

// WaitSeconds
// Simple async node that waits for a given number of seconds.
// Used to pause at waypoints (e.g. stop at the survivor for 1 second).

class WaitSeconds : public BT::StatefulActionNode
{
public:
  WaitSeconds(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;     // records the end time
  BT::NodeStatus onRunning() override;   // checks if time is up
  void onHalted() override;

private:
  // the point in time when the wait will be over
  std::chrono::steady_clock::time_point end_time_;
};

// PublishObjectTF
// Broadcasts a static TF frame for a detected object at a given x,y position.
// This is how we mark the survivor location on the map so other nodes can find it.
// SyncActionNode because it finishes in one tick (just publishes once).

class PublishObjectTF : public BT::SyncActionNode
{
public:
  PublishObjectTF(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  // tick() runs once and returns SUCCESS or FAILURE
  BT::NodeStatus tick() override;

private:
  void ensureInit();
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;  // publishes to /tf_static
};

// SetCameraAngle
// Publishes a Float64 command to the camera joint controller to rotate the camera.
// Used during the dam scan to look left, centre, right.

class SetCameraAngle : public BT::SyncActionNode
{
public:
  SetCameraAngle(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void ensureInit();
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;  // publishes to /camera_controller/commands
};

// PublishDockStatus
// Publishes a Bool on /is_docked to tell the battery simulator whether
// the robot is currently docked (charging) or undocked (draining).

class PublishDockStatus : public BT::SyncActionNode
{
public:
  PublishDockStatus(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void ensureInit();
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;  // publishes to /is_docked
};

// CheckBatteryOK
// Condition node that checks if the battery is okay.
// Subscribes to /battery_level_low and returns FAILURE if the battery is low,
// which causes the reactive fallback in the tree to trigger the DockAndRecharge subtree.

class CheckBatteryOK : public BT::ConditionNode
{
public:
  CheckBatteryOK(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void ensureInit();
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;  // subscribes to /battery_level_low
  bool battery_low_;  // updated by the subscription callback
};

// CheckFireSafe
// Condition node that checks if the robot is a safe distance from the fire.
// Uses TF2 to get the robot's current position and calculates the distance
// to the known fire coordinates. Returns FAILURE if too close.
// Wrapped in a ReactiveSequence at the top of the tree so it gets checked every tick.

class CheckFireSafe : public BT::ConditionNode
{
public:
  CheckFireSafe(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void ensureInit();
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;           // stores recent transforms
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;  // populates the buffer from /tf
};

// WaitForBatteryOK
// Async node that blocks until the battery is no longer low.
// Used in the DockAndRecharge subtree after docking, it just keeps returning
// RUNNING until the battery simulator says the battery is charged up again.

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
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;  // subscribes to /battery_level_low
  bool battery_low_;  // starts true, flips to false when battery is recharged
};

}  // namespace rescue_bt

#endif  // RESCUE_BT__BT_NODES_HPP_
