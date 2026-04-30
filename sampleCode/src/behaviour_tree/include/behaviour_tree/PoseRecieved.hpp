// Header guard: prevents this file from being included multiple times
#ifndef BEHAVIOUR_TREE_POSERECIEVED_HPP
#define BEHAVIOUR_TREE_POSERECIEVED_HPP

// Include the BehaviorTree.CPP library's action_node base class
#include <behaviortree_cpp_v3/action_node.h>
// Include ROS2 client library for node and subscription functionality
#include <rclcpp/rclcpp.hpp>
// Include ROS2 message type for pose data (position + orientation with timestamp)
#include <geometry_msgs/msg/pose_stamped.hpp>

// Put our class in the behaviour_tree namespace
namespace behaviour_tree
{
// PoseRecieved is a BehaviorTree condition/action that checks if a pose message was received
// It inherits from SyncActionNode (NOT StatefulActionNode) because it completes instantly
// SyncActionNode = Synchronous = finishes in a single tick (no waiting across multiple ticks)
// Use case: Wait for a goal pose to be published before planning a path
class PoseRecieved : public BT::SyncActionNode
{
public:
    // Constructor: called when the node is created from the XML tree
    // name: the name given to this node instance in the XML
    // config: configuration containing blackboard access and port definitions
    PoseRecieved(const std::string& name, const BT::NodeConfiguration& config);
    
    // Static method that defines what input/output ports this node has
    // This node takes IN a topic name and outputs the received pose
    static BT::PortsList providedPorts()
    {
        return { 
            // INPUT: the name of the ROS2 topic to subscribe to (e.g., "/goal_pose")
            BT::InputPort<std::string>("topic_name"),
            // OUTPUT: the latest pose message received from that topic
            // Other nodes can read this from the blackboard to get the pose data
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose") 
        };
    }
    
    // tick() is called each time the behavior tree executes this node
    // For SyncActionNode, this must complete and return a status immediately
    // Returns: SUCCESS if pose received, FAILURE if not received yet
    BT::NodeStatus tick() override;
    
private:
    // ROS2 node pointer - needed for creating subscriptions and ROS2 communication
    rclcpp::Node::SharedPtr node_;
    
    // ROS2 subscription - listens to a topic and calls our callback when messages arrive
    // Subscribes to PoseStamped messages
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    
    // Storage for the most recently received pose message
    // Updated each time the poseCallback is triggered
    geometry_msgs::msg::PoseStamped latest_pose_;
    
    // Flag: true if we've received at least one pose message, false otherwise
    // Initialized to false - no pose received yet when node is created
    bool pose_received_ = false;
    
    // Callback function: automatically called by ROS2 when a new pose message arrives
    // This is where we store the incoming pose and set the pose_received_ flag
    // msg: shared pointer to the received PoseStamped message
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};

}  // namespace behaviour_tree

// End of header guard
#endif  // BEHAVIOUR_TREE_POSERECIEVED_HPP