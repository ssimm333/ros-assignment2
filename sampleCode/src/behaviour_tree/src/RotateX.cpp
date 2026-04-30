// Include our header file that declares the RotateX class
#include "behaviour_tree/RotateX.hpp"

// Put implementation in the behaviour_tree namespace
namespace behaviour_tree
{
// Constructor implementation: sets up the ROS2 action client and initializes state flags
// This is called once when the behavior tree is loaded from XML
RotateX::RotateX(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)  // Call base class constructor
{
    // Get the ROS2 node from the blackboard (shared memory for all BT nodes)
    // This gives us access to ROS2 functionality
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    
    // Create a ROS2 action client for the "rotate_x" action
    // This will communicate with an action server that controls the robot's rotation
    // The action server must be running for this to work
    action_client_ = rclcpp_action::create_client<RotateXAction>(node_, "rotate_x");
    
    // Initialize all state flags to their default values
    // These track the lifecycle of the rotation action goal
    goal_active_ = false;      // No goal is currently being executed
    goal_sent_ = false;        // We haven't sent a goal yet
    success_ = false;          // The goal hasn't succeeded yet
    current_angle_ = 0.0;      // Current rotation angle (updated from feedback)
    angle_diff_ = 0.0;         // Track change in angle (for progress monitoring)
}

// onStart() is called THE FIRST TIME this node is ticked by the behavior tree
// This is where we set up and send the rotation action goal to the robot
BT::NodeStatus RotateX::onStart()
{
    // Reset success flag at the start of each execution
    success_ = false;
    
    // Check if the required "angle" input port has a value
    // has_value() returns true if the optional contains a value
    if(!getInput<double>("angle").has_value()) {
        // Log error - this input is mandatory, can't rotate without knowing how much
        RCLCPP_ERROR(node_->get_logger(), "RotateX: Missing required input: angle");
        return BT::NodeStatus::FAILURE;
    }

    // Read input values from the ports (which read from the blackboard)
    // Note: tolerance_ is read first in this code, but order doesn't matter
    tolerance_ = getInput<double>("tolerance").value_or(0.1);   // Optional, default 0.1 radians
    angle_ = getInput<double>("angle").value();                 // Required, already checked above
    speed_ = getInput<double>("speed").value_or(0.3);          // Optional, default 0.3 rad/s
    
    // Log the parameters we're using (helpful for debugging)
    RCLCPP_INFO(node_->get_logger(), "RotateX: Starting with angle: %f, speed: %f, tolerance: %f", angle_, speed_, tolerance_);
    
    // Wait up to 5 seconds for the action server to become available
    // The action server is the separate node that actually controls the robot
    if(!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
        // Action server is not available - maybe it's not running?
        RCLCPP_ERROR(node_->get_logger(), "RotateX: Action server not available after waiting");
        return BT::NodeStatus::FAILURE;
    } 

    // Create the action goal message with our rotation parameters
    RotateXAction::Goal goal;
    goal.angle = angle_;                  // How much to rotate (in radians)
    goal.rotation_speed = speed_;         // How fast to rotate (rad/s)
    goal.tolerance = tolerance_;          // How close is close enough (radians)
    
    // Create options struct to configure callbacks for the async action
    auto send_goal_options = rclcpp_action::Client<RotateXAction>::SendGoalOptions();
    
    // Set callback for when server responds to our goal request
    // This lambda function will be called when the server accepts or rejects the goal
    // [this] captures 'this' pointer so we can access member variables
    send_goal_options.goal_response_callback = [this](GoalHandleRotateX::SharedPtr goal_handle) {
        // Clear the "goal_sent" flag since we now have a response
        goal_sent_ = false;
        
        // Check if goal was rejected (goal_handle will be null)
        if(!goal_handle) {
            RCLCPP_ERROR(rclcpp::get_logger("RotateX"), "RotateX: Goal was rejected by the action server");
            goal_active_ = false;
        } else {
            // Goal was accepted! Save the handle and mark as active
            RCLCPP_INFO(rclcpp::get_logger("RotateX"), "RotateX: Goal accepted by the action server, waiting for result");
            goal_active_ = true;
            goal_handle_ = goal_handle;  // Store handle so we can cancel later if needed
        }
    };
    
    // Set callback for when the rotation action completes (success or failure)
    // This lambda is called when the action server finishes executing our goal
    send_goal_options.result_callback = [this](const GoalHandleRotateX::WrappedResult& result) {
        // Goal is no longer active (it finished)
        goal_active_ = false;
        
        // Check if the result indicates success
        // result.result is the custom result message defined in the action interface
        if(result.result && result.result->success) {
            RCLCPP_INFO(node_->get_logger(), "RotateX: Goal succeeded");
            success_ = true;   // Mark success so onRunning() returns SUCCESS
        } else {
            RCLCPP_ERROR(node_->get_logger(), "RotateX: Goal failed");
            success_ = false;  // Mark failure so onRunning() returns FAILURE
        }
    };

    // Set callback for periodic feedback while rotation action is executing
    // This is called regularly by the action server to report progress
    // Currently empty, but could be used to monitor rotation progress or detect stuck robot
    send_goal_options.feedback_callback = [this](GoalHandleRotateX::SharedPtr, const std::shared_ptr<const RotateXAction::Feedback> feedback) {
        // Could track feedback here, e.g., current_angle_ = feedback->current_angle
    };
    
    // Actually send the goal to the action server (asynchronously)
    // This returns a future, but we don't wait for it - the callbacks handle the response
    auto future_goal_handle = action_client_->async_send_goal(goal, send_goal_options);
    
    // Mark that we've sent the goal (so onRunning knows we're waiting for response)
    goal_sent_ = true;
    
    // Return RUNNING to tell the behavior tree this action is in progress
    // onRunning() will be called on subsequent ticks to check status
    return BT::NodeStatus::RUNNING;
}

// onRunning() is called on EVERY SUBSEQUENT TICK after onStart()
// This checks the current state of the rotation action and returns appropriate status
// The behavior tree will keep calling this until it returns SUCCESS or FAILURE
BT::NodeStatus RotateX::onRunning()
{    
    // Phase 1: Still waiting for the action server to accept/reject our goal
    // goal_sent_ is true from when we called async_send_goal, becomes false when callback runs
    if(goal_sent_) {
        return BT::NodeStatus::RUNNING;  // Keep waiting for goal response
    }
    
    // Phase 2: Goal was accepted and is currently being executed by the action server
    // goal_active_ is set to true in goal_response_callback, becomes false in result_callback
    if(goal_active_) {
        return BT::NodeStatus::RUNNING;  // Keep waiting for rotation to complete
    }
    
    // Phase 3: Goal has completed (result_callback was called)
    // Check the success_ flag that was set by the result_callback
    // Return SUCCESS if robot completed the rotation, FAILURE if it failed
    return success_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// onHalted() is called if the behavior tree decides to interrupt/stop this action
// This can happen if a parent node (like a Sequence) is halted or a higher priority
// behavior needs to take control
void RotateX::onHalted()
{
    // Check if we have an active goal that needs to be cancelled
    if(goal_active_ && goal_handle_) {
        // Send cancel request to the action server to stop the robot's rotation
        action_client_->async_cancel_goal(goal_handle_);
        RCLCPP_INFO(node_->get_logger(), "RotateX: Goal cancelled");
    }
    
    // Set the node's status back to IDLE (ready for next execution)
    // This is important for StatefulActionNode to reset properly
    setStatus(BT::NodeStatus::IDLE);
}

} // namespace behaviour_tree