// Include our header file that declares the GetAlignRotation class
#include <behaviour_tree/GetAlignRotation.hpp>

// Put implementation in the behaviour_tree namespace
namespace behaviour_tree
{
// Constructor implementation: minimal setup needed for this simple calculation node
// This is called once when the behavior tree is loaded from XML
GetAlignRotation::GetAlignRotation(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)  // Call base class constructor
{
    // No initialization needed - this is a stateless calculation node
    // It doesn't need to store any ROS2 nodes or subscriptions
}

// tick() is called by the behavior tree every time it evaluates this node
// This calculates the angle needed to face a target position
BT::NodeStatus GetAlignRotation::tick()
{
    // Try to read the target pose from the INPUT port "target_pose"
    // This was likely written to the blackboard by a PoseRecieved node
    // getInput returns std::optional - it may or may not have a value
    auto target_pose = getInput<geometry_msgs::msg::PoseStamped>("target_pose");
    
    // Check if the input was successfully retrieved
    if (!target_pose)
    {
        // Log an error if the required input is missing
        // This helps with debugging - maybe the port isn't connected in the XML
        RCLCPP_ERROR(rclcpp::get_logger("GetAlignRotation"), "Missing required input: target_pose");
        
        // Return FAILURE - we can't calculate without a target pose
        return BT::NodeStatus::FAILURE;
    }

    // Calculate the angle to face the target using atan2
    // atan2(y, x) returns the angle in radians from the origin to point (x, y)
    // This assumes the robot is at the origin (0, 0) - in a real system you'd
    // get the current robot pose from TF and calculate relative to that
    // Result is in range [-PI, PI] where 0 is facing +X axis
    double align_rotation = std::atan2(target_pose->pose.position.y, target_pose->pose.position.x);
    
    // Write the calculated angle to the OUTPUT port "align_rotation"
    // This makes it available on the blackboard for other nodes (like RotateX) to use
    setOutput("align_rotation", align_rotation);
    
    // Return SUCCESS - calculation completed successfully
    return BT::NodeStatus::SUCCESS;
}

}  // namespace behaviour_tree