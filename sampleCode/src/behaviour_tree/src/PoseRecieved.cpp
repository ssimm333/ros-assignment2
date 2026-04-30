// Include our header file that declares the PoseRecieved class
#include <behaviour_tree/PoseRecieved.hpp>

// Put implementation in the behaviour_tree namespace (matching the header)
namespace behaviour_tree
{
// Constructor implementation: sets up the node and creates the ROS2 subscription
// This is called once when the behavior tree is loaded from XML
PoseRecieved::PoseRecieved(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)  // Call base class constructor
{
    // Get the ROS2 node from the blackboard (shared memory for all BT nodes)
    // The blackboard was populated in bt_runner.cpp with: blackboard->set("node", node)
    // This gives us access to ROS2 functionality like logging and creating subscriptions
    node_= config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    
    // Initialize the flag to false - we haven't received any pose yet
    pose_received_ = false;
    
    // Try to get the topic name from the input port (defined in XML)
    // If not provided, use default "pose_topic"
    // value_or() returns the default if getInput returns empty/null
    std::string topic_name_ = getInput<std::string>("topic_name").value_or("pose_topic");
    
    // Log which topic we're subscribing to (helpful for debugging)
    RCLCPP_INFO(node_->get_logger(), "Subscribing to topic: %s", topic_name_.c_str());
    
    // Create a ROS2 subscription to listen for PoseStamped messages
    // Parameters: topic_name, queue_size (10), callback function
    // std::bind creates a callback that calls our poseCallback method
    // std::placeholders::_1 means "pass the first argument (the message) to poseCallback"
    subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic_name_,10, std::bind(&PoseRecieved::poseCallback, this, std::placeholders::_1));
}

// Callback function: automatically called by ROS2 whenever a new pose message arrives
// This runs in the background (asynchronously) whenever a message is published
void PoseRecieved::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // Dereference the shared pointer and store the pose data
    // SharedPtr is a smart pointer, * dereferences it to get the actual message
    latest_pose_ = *msg;
    
    // Set flag to true indicating we have new data
    // This will be checked by tick() to determine if the node succeeds
    pose_received_ = true;
}

// tick() is called by the behavior tree every time it evaluates this node
// For SyncActionNode, this must return immediately (no waiting/blocking)
BT::NodeStatus PoseRecieved::tick()
{
    // Check if we've received a pose since the last tick
    if (pose_received_)
    {
        // Log the received pose coordinates for debugging
        RCLCPP_INFO(node_->get_logger(), "Pose received: [%.2f, %.2f]", 
                    latest_pose_.pose.position.x, 
                    latest_pose_.pose.position.y);
        
        // Write the pose to the OUTPUT port "pose"
        // This makes the data available on the blackboard for other nodes to read
        // Other nodes can access this via their INPUT ports
        setOutput("pose", latest_pose_);
        
        // Reset flag so we wait for a new pose next time
        // This ensures we only return SUCCESS once per received message
        pose_received_ = false; 
        
        // Return SUCCESS - we have successfully received and output a pose
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        // Return FAILURE - no new pose has been received yet
        // The behavior tree might retry this node or take a different branch
        return BT::NodeStatus::FAILURE;
    }
}

}  // namespace behaviour_tree