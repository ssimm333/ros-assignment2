// Include our header file that declares the GetDistance class
#include <behaviour_tree/GetDistance.hpp>

// Put implementation in the behaviour_tree namespace
namespace behaviour_tree
{
// Constructor implementation: minimal setup needed for this simple calculation node
// This is called once when the behavior tree is loaded from XML
GetDistance::GetDistance(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)  // Call base class constructor
{
    // No initialization needed - this is a stateless calculation node
    // It doesn't need to store any ROS2 nodes or subscriptions
}

// tick() is called by the behavior tree every time it evaluates this node
// This calculates the Euclidean distance from the robot to a target position
BT::NodeStatus GetDistance::tick()
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
        RCLCPP_ERROR(rclcpp::get_logger("GetDistance"), "Missing required input: target_pose");
        
        // Return FAILURE - we can't calculate without a target pose
        return BT::NodeStatus::FAILURE;
    }   

    // Calculate the Euclidean distance using the Pythagorean theorem
    // distance = sqrt(x^2 + y^2)
    // std::pow(value, 2) squares the value
    // This assumes the robot is at the origin (0, 0) - in a real system you'd
    // get the current robot pose from TF and calculate relative distance
    double distance = std::sqrt(std::pow(target_pose->pose.position.x, 2) + std::pow(target_pose->pose.position.y, 2));
    
    // Write the calculated distance to the OUTPUT port "distance"
    // This makes it available on the blackboard for other nodes (like DriveX) to use
    setOutput("distance", distance);
    
    // Return SUCCESS - calculation completed successfully
    return BT::NodeStatus::SUCCESS;
}

}  // namespace behaviour_tree  