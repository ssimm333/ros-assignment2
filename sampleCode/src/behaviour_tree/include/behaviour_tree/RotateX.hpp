// Header guard: prevents this file from being included multiple times in the same compilation
#ifndef   BEHAVIOUR_TREE_ROTATEX_HPP
#define   BEHAVIOUR_TREE_ROTATEX_HPP

// Include the BehaviorTree.CPP library's action_node base class
#include <behaviortree_cpp_v3/action_node.h>

// Include ROS2 client library for node functionality
#include <rclcpp/rclcpp.hpp>
// Include ROS2 action client library for sending action goals and monitoring progress
#include <rclcpp_action/rclcpp_action.hpp>

// Include our custom action interface definition (defines the RotateX action message types)
#include "bt_interfaces/action/rotate_x.hpp"

// Put our class in the behaviour_tree namespace to avoid naming conflicts
namespace behaviour_tree
{
// RotateX is a BehaviorTree action node that commands a robot to rotate by a specified angle
// It inherits from StatefulActionNode because rotation takes time (asynchronous operation)
// StatefulActionNode allows the action to span multiple behavior tree "ticks"
// Very similar to DriveX, but for rotational movement instead of linear movement
class RotateX : public BT::StatefulActionNode
{
public:
    // Type alias for our ROS2 action type - makes code more readable
    // This is the action message type defined in bt_interfaces package
    using RotateXAction = bt_interfaces::action::RotateX;
    
    // Type alias for the goal handle returned when we send an action goal
    // The goal handle lets us check status and get results from the action server
    using GoalHandleRotateX = rclcpp_action::ClientGoalHandle<RotateXAction>;
    
    // Constructor: called when the node is created from the XML tree
    // name: the name given to this node instance in the XML
    // config: configuration containing blackboard access and port definitions
    RotateX(const std::string& name, const BT::NodeConfiguration& config);
    
    // onStart() is called the FIRST time this node is ticked
    // This is where we initialize and send the rotation action goal to the robot
    // Returns: RUNNING if action sent, FAILURE if something went wrong
    BT::NodeStatus onStart() override;
    
    // onRunning() is called on every subsequent tick while the action is running
    // This checks if the rotation has completed and returns the appropriate status
    // Returns: RUNNING if still rotating, SUCCESS if done, FAILURE if error
    BT::NodeStatus onRunning() override;
    
    // onHalted() is called if the behavior tree decides to stop/interrupt this action
    // This is where we clean up and cancel any ongoing rotation goals
    void onHalted() override;
    
    // Static method that defines what input/output ports this node has
    // Ports are how data flows between nodes through the blackboard
    static BT::PortsList providedPorts()
    {
        // Return a list of three INPUT ports (data coming INTO this node)
        return { 
            // angle: how much to rotate in radians (positive = counter-clockwise, negative = clockwise)
            BT::InputPort<double>("angle"),
            // speed: how fast to rotate in radians per second
            BT::InputPort<double>("speed"),
            // tolerance: acceptable error margin in radians (how close is "close enough")
            BT::InputPort<double>("tolerance") 
        };
    }
    
private:
    // ROS2 action client - sends rotation action goals to the robot's action server
    // SharedPtr is a smart pointer that handles memory management automatically
    rclcpp_action::Client<RotateXAction>::SharedPtr action_client_;
    
    // ROS2 node pointer - needed for ROS2 communication (spinning, logging, etc.)
    rclcpp::Node::SharedPtr node_;

    // Handle to the currently active rotation action goal
    // Used to check goal status and get results
    GoalHandleRotateX::SharedPtr goal_handle_;

    // Cached value: the angle to rotate (read from the "angle" input port)
    double angle_;
    
    // Cached value: the rotational speed (read from the "speed" input port)
    double speed_;
    
    // Cached value: the acceptable error tolerance (read from the "tolerance" input port)
    double tolerance_;
    
    // Tracks the last reported angle from feedback - used to detect if robot is stuck
    double last_angle_;
    
    // The difference between current and last angle - helps detect rotation progress
    double angle_diff_;
    
    // Flag: true if we currently have an active goal on the action server
    bool goal_active_;
    
    // Flag: true if we've already sent the goal (prevents sending multiple times)
    bool goal_sent_;
    
    // Flag: true if the rotation action completed successfully
    bool success_;
    
    // Stores the current angle during rotation - updated from action feedback
    double current_angle_;

};
    
} // namespace behaviour_tree

// End of header guard - closes the #ifndef at the top
#endif // BEHAVIOUR_TREE_ROTATEX_HPP