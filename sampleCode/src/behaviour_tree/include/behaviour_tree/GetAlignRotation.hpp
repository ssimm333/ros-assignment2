// Header guard: prevents this file from being included multiple times
#ifndef BEHAVIOUR_TREE_GETALIGNROTATION_HPP
#define BEHAVIOUR_TREE_GETALIGNROTATION_HPP

// Include the BehaviorTree.CPP library's action_node base class
#include <behaviortree_cpp_v3/action_node.h>
// Include ROS2 client library (may be needed for logging or node access)
#include <rclcpp/rclcpp.hpp>
// Include ROS2 message type for pose data (position + orientation with timestamp)
#include <geometry_msgs/msg/pose_stamped.hpp>

// Put our class in the behaviour_tree namespace
namespace behaviour_tree
{
// GetAlignRotation is a BehaviorTree action that calculates the rotation needed to face a target
// It inherits from SyncActionNode because the calculation is instant (no waiting)
// SyncActionNode = Synchronous = finishes in a single tick
// Use case: Calculate how much to rotate before driving toward a goal position
// This is typically used in sequence: GetAlignRotation -> RotateX -> GetDistance -> DriveX
class GetAlignRotation : public BT::SyncActionNode
{
public:
    // Constructor: called when the node is created from the XML tree
    // name: the name given to this node instance in the XML
    // config: configuration containing blackboard access and port definitions
    GetAlignRotation(const std::string& name, const BT::NodeConfiguration& config);
    
    // tick() is called each time the behavior tree executes this node
    // For SyncActionNode, this must complete and return a status immediately
    // This calculates the angle needed to face the target pose
    // Returns: SUCCESS with the calculated rotation, or FAILURE if calculation failed
    BT::NodeStatus tick() override;
    
    // Static method that defines what input/output ports this node has
    // Takes IN a target pose, calculates and outputs the rotation angle needed
    static BT::PortsList providedPorts()
    {
        return { 
            // INPUT: the target pose we want to face/align towards
            // This is typically read from the blackboard (set by PoseRecieved or similar)
            BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
            
            // OUTPUT: the rotation angle (in radians) needed to face the target
            // This can be fed into a RotateX node to actually perform the rotation
            // Calculated using atan2(dy, dx) from robot's current position to target
            BT::OutputPort<double>("align_rotation")
        };
    }
    
private:
    // No private member variables needed - this is a pure calculation node
    // It reads the current robot pose from TF (transform tree) and computes the angle
};

}  // namespace behaviour_tree

// End of header guard
#endif  // BEHAVIOUR_TREE_GETALIGNROTATION_HPP