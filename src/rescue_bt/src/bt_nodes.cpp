// bt_nodes.cpp
// Implementation of all the custom BT nodes declared in bt_nodes.hpp.
// Each node interacts with ROS2 in some way (action clients, publishers,
// subscribers, TF lookups) and exposes that functionality to the behavior tree.

#include "rescue_bt/bt_nodes.hpp"

#include <cmath>

#include "geometry_msgs/msg/transform_stamped.hpp"  // used for building TF messages
#include "tf2/LinearMath/Quaternion.h"               // quaternion math for yaw to orientation conversion

namespace rescue_bt
{

// ============ NavigateToWaypoint ============

// constructor just initialises the tracking flags to their default state
NavigateToWaypoint::NavigateToWaypoint(
  const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config),
  goal_done_(false),
  goal_succeeded_(false),
  goal_sent_(false)
{
}

// lazy init: we cant do this in the constructor because the blackboard
// isnt populated yet when BT.CPP constructs the nodes
void NavigateToWaypoint::ensureInit()
{
  if (!node_) {
    // grab the shared ROS node that was put on the blackboard in main()
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    // create an action client for the Nav2 navigate_to_pose action
    action_client_ = rclcpp_action::create_client<NavigateToPose>(
      node_, "navigate_to_pose");
  }
}

// these ports map directly to the XML attributes, e.g. x="15.1" y="11.9"
BT::PortsList NavigateToWaypoint::providedPorts()
{
  return {
    BT::InputPort<double>("x"),
    BT::InputPort<double>("y"),
    BT::InputPort<double>("theta", 0.0, "Goal orientation (yaw)"),
    BT::InputPort<std::string>("description", "", "Waypoint label for logging"),
  };
}

// onStart is called once when the tree first ticks this node
// it reads the waypoint coords from the ports and sends the nav goal
BT::NodeStatus NavigateToWaypoint::onStart()
{
  ensureInit();

  // read the target coordinates from the XML ports
  double x, y, theta;
  if (!getInput("x", x) || !getInput("y", y)) {
    RCLCPP_ERROR(node_->get_logger(), "NavigateToWaypoint: missing x or y port");
    return BT::NodeStatus::FAILURE;
  }
  getInput("theta", theta);

  std::string desc;
  getInput("description", desc);
  RCLCPP_INFO(node_->get_logger(), "Navigating to %s (%.1f, %.1f)", desc.c_str(), x, y);

  // wait up to 10 seconds for Nav2 to be ready
  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node_->get_logger(), "navigate_to_pose action server not available");
    return BT::NodeStatus::FAILURE;
  }

  // reset the state flags before sending a new goal
  goal_done_ = false;
  goal_succeeded_ = false;
  goal_sent_ = false;

  return sendGoal(x, y, theta, desc);
}

// sendGoal builds a NavigateToPose goal message and sends it to Nav2 asynchronously.
// if anything goes wrong (timeout, rejection) it returns RUNNING so onRunning can retry.
BT::NodeStatus NavigateToWaypoint::sendGoal(
  double x, double y, double theta, const std::string & desc)
{
  // build the goal message with the target pose in the map frame
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = node_->now();
  goal_msg.pose.pose.position.x = x;
  goal_msg.pose.pose.position.y = y;

  // convert yaw angle (theta) to a quaternion for the orientation
  // setRPY takes roll, pitch, yaw and we only care about yaw here
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  goal_msg.pose.pose.orientation.x = q.x();
  goal_msg.pose.pose.orientation.y = q.y();
  goal_msg.pose.pose.orientation.z = q.z();
  goal_msg.pose.pose.orientation.w = q.w();

  // set up a callback that fires when Nav2 finishes the navigation
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
    [this](const GoalHandle::WrappedResult & result) {
      goal_done_ = true;
      goal_succeeded_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
    };

  // send the goal and wait up to 5 seconds for the server to acknowledge it
  auto future = action_client_->async_send_goal(goal_msg, send_goal_options);
  if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    // send timed out, return RUNNING so the tree retries on the next tick
    RCLCPP_WARN(node_->get_logger(), "Failed to send navigation goal, will retry...");
    return BT::NodeStatus::RUNNING;
  }

  goal_handle_ = future.get();
  if (!goal_handle_) {
    // goal was rejected by Nav2, maybe it hasnt fully activated yet
    RCLCPP_WARN(node_->get_logger(), "Navigation goal rejected (Nav2 may still be activating), will retry...");
    return BT::NodeStatus::RUNNING;
  }

  // goal was accepted, now we wait for the result callback to fire
  goal_sent_ = true;
  RCLCPP_INFO(node_->get_logger(), "Navigation goal accepted for %s", desc.c_str());
  return BT::NodeStatus::RUNNING;
}

// onRunning gets called every tick while the node is still RUNNING
BT::NodeStatus NavigateToWaypoint::onRunning()
{
  if (!goal_sent_) {
    // goal wasnt accepted last time, try sending it again
    double x, y, theta;
    std::string desc;
    getInput("x", x);
    getInput("y", y);
    getInput("theta", theta);
    getInput("description", desc);
    return sendGoal(x, y, theta, desc);
  }
  if (goal_done_) {
    // Nav2 finished, return whether it succeeded or failed
    return goal_succeeded_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  // still navigating, keep waiting
  return BT::NodeStatus::RUNNING;
}

// onHalted is called if the tree interrupts this node (e.g. fire safety triggers)
// we need to cancel the active Nav2 goal so the robot stops moving
void NavigateToWaypoint::onHalted()
{
  if (goal_handle_) {
    action_client_->async_cancel_goal(goal_handle_);
    RCLCPP_INFO(node_->get_logger(), "NavigateToWaypoint: navigation cancelled");
  }
}

// ============ WaitSeconds ============

WaitSeconds::WaitSeconds(
  const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

// takes a "seconds" port, defaults to 1 second if not specified
BT::PortsList WaitSeconds::providedPorts()
{
  return {
    BT::InputPort<double>("seconds", 1.0, "Duration to wait"),
  };
}

// record when we should stop waiting
BT::NodeStatus WaitSeconds::onStart()
{
  double seconds;
  getInput("seconds", seconds);
  // convert seconds to milliseconds and calculate the end time
  end_time_ = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(static_cast<int>(seconds * 1000));
  return BT::NodeStatus::RUNNING;
}

// keep returning RUNNING until the clock passes our end time
BT::NodeStatus WaitSeconds::onRunning()
{
  if (std::chrono::steady_clock::now() >= end_time_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

// nothing to clean up for a simple timer
void WaitSeconds::onHalted()
{
}

// ============ PublishObjectTF ============
// publishes a static transform so the object (e.g. survivor) shows up in the TF tree

PublishObjectTF::PublishObjectTF(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

void PublishObjectTF::ensureInit()
{
  if (!node_) {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    // static broadcaster persists transforms even after this node finishes
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  }
}

// frame_name is the name that will appear in the TF tree, x and y are map coordinates
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

  // build a transform from "map" to the named frame at the given position
  // z is 0 because everything is on the ground plane
  // rotation w=1 means no rotation (identity quaternion)
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

// ============ SetCameraAngle ============
// sends a command to the camera joint controller to point the camera
// used during the dam scan sequence to sweep left/centre/right

SetCameraAngle::SetCameraAngle(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

void SetCameraAngle::ensureInit()
{
  if (!node_) {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    // the camera controller expects Float64 commands on this topic
    pub_ = node_->create_publisher<std_msgs::msg::Float64>(
      "/camera_controller/commands", 10);
  }
}

// angle is in radians, 0 is straight ahead, positive is left, negative is right
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

  // publish the angle command to move the camera joint
  auto msg = std_msgs::msg::Float64();
  msg.data = angle;
  pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "Camera angle set to %.4f rad", angle);
  return BT::NodeStatus::SUCCESS;
}

// ============ PublishDockStatus ============
// tells the battery simulator whether the robot is docked or not
// when docked=true, the battery simulator switches from draining to charging

PublishDockStatus::PublishDockStatus(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

void PublishDockStatus::ensureInit()
{
  if (!node_) {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    // the battery simulator subscribes to this topic
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

  // publish the dock status so the battery sim knows to charge or drain
  auto msg = std_msgs::msg::Bool();
  msg.data = docked;
  pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "Dock status: %s", docked ? "DOCKED" : "UNDOCKED");
  return BT::NodeStatus::SUCCESS;
}

// ============ CheckBatteryOK ============
// condition node that returns SUCCESS if battery is fine, FAILURE if low.
// when this returns FAILURE inside a ReactiveFallback, the fallback
// triggers the DockAndRecharge subtree to go charge.

CheckBatteryOK::CheckBatteryOK(
  const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config),
  battery_low_(false)  // assume battery is fine at startup
{
}

void CheckBatteryOK::ensureInit()
{
  if (!node_) {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    // subscribe to the battery simulator's output topic
    // the callback just stores the latest value so tick() can check it
    sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/battery_level_low", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        battery_low_ = msg->data;
      });
  }
}

// no ports needed, we get battery status from the subscription not the XML
BT::PortsList CheckBatteryOK::providedPorts()
{
  return {};
}

BT::NodeStatus CheckBatteryOK::tick()
{
  ensureInit();
  if (battery_low_) {
    // throttled so it doesnt spam the log every tick (every 5 seconds max)
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "CheckBatteryOK: battery is LOW — triggering dock");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

// ============ CheckFireSafe ============
// safety condition that checks the robot isnt too close to the fire.
// sits at the top of the main tree inside a ReactiveSequence so it
// gets evaluated every single tick regardless of what else is happening.

// known fire location in the world (from the assignment world file)
static constexpr double FIRE_X = -14.2;
static constexpr double FIRE_Y = 10.8;
// minimum safe distance in metres
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
    // TF buffer stores recent transforms, the listener populates it from /tf
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

  // look up where the robot is right now relative to the map
  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    // if we cant get the transform, assume its safe (dont block the mission)
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "CheckFireSafe: cannot get robot pose: %s", ex.what());
    return BT::NodeStatus::SUCCESS;
  }

  // calculate euclidean distance from the robot to the fire
  double dx = t.transform.translation.x - FIRE_X;
  double dy = t.transform.translation.y - FIRE_Y;
  double dist = std::sqrt(dx * dx + dy * dy);

  if (dist < FIRE_SAFE_DISTANCE) {
    // too close, returning FAILURE will halt the whole mission via the ReactiveSequence
    RCLCPP_WARN(node_->get_logger(), "TOO CLOSE TO FIRE! dist=%.2f m", dist);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

// ============ WaitForBatteryOK ============
// used in the DockAndRecharge subtree after the robot has docked.
// keeps returning RUNNING until the battery simulator says the battery
// is no longer low (i.e. its charged past the full_threshold).

WaitForBatteryOK::WaitForBatteryOK(
  const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config),
  battery_low_(true)  // start assuming low so we dont skip the wait
{
}

void WaitForBatteryOK::ensureInit()
{
  if (!node_) {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    // same topic as CheckBatteryOK, but this node waits for it to go false
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

// if the battery is already fine when we start, skip straight to SUCCESS
BT::NodeStatus WaitForBatteryOK::onStart()
{
  ensureInit();
  RCLCPP_INFO(node_->get_logger(), "Waiting for battery to recharge...");
  return battery_low_ ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
}

// keep polling until battery_low_ flips to false
BT::NodeStatus WaitForBatteryOK::onRunning()
{
  return battery_low_ ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
}

// nothing to clean up
void WaitForBatteryOK::onHalted()
{
}

}  // namespace rescue_bt
