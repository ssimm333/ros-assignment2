// Header guard: prevents this file from being included multiple times
#ifndef BEHAVIOUR_TREE_GETDISTANCE_HPP
#define BEHAVIOUR_TREE_GETDISTANCE_HPP

// Include the BehaviorTree.CPP library's action_node base class
#include <behaviortree_cpp_v3/action_node.h>
// Include ROS2 client library (may be needed for logging or node access)
#include <rclcpp/rclcpp.hpp>
// Include ROS2 message type for pose data (position + orientation with timestamp)
#include <geometry_msgs/msg/pose_stamped.hpp>

// Put our class in the behaviour_tree namespace
namespace behaviour_tree
{
// GetDistance is a BehaviorTree action that calculates the distance to a target position
// It inherits from SyncActionNode because the calculation is instant (no waiting)
// SyncActionNode = Synchronous = finishes in a single tick
// Use case: Calculate how far to drive after aligning rotation toward a goal
// This is typically used in sequence: GetAlignRotation -> RotateX -> GetDistance -> DriveX
class GetDistance : public BT::SyncActionNode
{
public:
    // Constructor: called when the node is created from the XML tree
    // name: the name given to this node instance in the XML
    // config: configuration containing blackboard access and port definitions
    GetDistance(const std::string& name, const BT::NodeConfiguration& config);
    
    // tick() is called each time the behavior tree executes this node
    // For SyncActionNode, this must complete and return a status immediately
    // This calculates the Euclidean distance from robot's current position to target
    // Returns: SUCCESS with the calculated distance, or FAILURE if calculation failed
    BT::NodeStatus tick() override;
    
    // Static method that defines what input/output ports this node has
    // Takes IN a target pose, calculates and outputs the distance to it
    static BT::PortsList providedPorts()
    {
        return { 
            // INPUT: the target pose we want to calculate distance to
            // This is typically read from the blackboard (set by PoseRecieved or similar)
            BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
            
            // OUTPUT: the distance (in meters) from robot's current position to target
            // This can be fed into a DriveX node to drive that distance
            // Calculated using: sqrt((x2-x1)^2 + (y2-y1)^2)
            BT::OutputPort<double>("distance")
        };
    }
    
private:
    // No private member variables needed - this is a pure calculation node
    // It reads the current robot pose from TF (transform tree) and computes the distance
};

}  // namespace behaviour_tree

// End of header guard
#endif  // BEHAVIOUR_TREE_GETDISTANCE_HPP