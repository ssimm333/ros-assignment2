// Include BehaviorTree.CPP library headers
#include <behaviortree_cpp_v3/bt_factory.h>          // Factory for creating BT nodes
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>  // Logger for debugging BT execution

// Include ROS2 headers
#include <rclcpp/rclcpp.hpp>  // Core ROS2 C++ client library
#include <ament_index_cpp/get_package_share_directory.hpp>  // To find package files

// Include our custom behavior tree node headers
#include "behaviour_tree/GetDistance.hpp"
#include "behaviour_tree/GetAlignRotation.hpp"
#include "behaviour_tree/PoseRecieved.hpp"
#include "behaviour_tree/DriveX.hpp"
#include "behaviour_tree/RotateX.hpp"

// Main entry point for the behavior tree runner
int main(int argc, char** argv)
{
    // Initialize ROS2 - must be called before any ROS2 functionality
    // This sets up the ROS2 middleware and allows node creation
    rclcpp::init(argc, argv);
    
    // Create a ROS2 node named "bt_runner"
    // This node will be used by all BT nodes for ROS2 communication
    // We share this node via the blackboard rather than creating separate nodes
    auto node = rclcpp::Node::make_shared("bt_runner");

    // Create the BehaviorTree factory
    // The factory is responsible for instantiating nodes when loading from XML
    BT::BehaviorTreeFactory factory;
    
    // Register all our custom node types with the factory
    // The string (e.g., "GetDistance") is how the node is referenced in the XML file
    // The template parameter is the C++ class to instantiate
    factory.registerNodeType<behaviour_tree::GetDistance>("GetDistance");
    factory.registerNodeType<behaviour_tree::GetAlignRotation>("GetAlignRotation");
    factory.registerNodeType<behaviour_tree::PoseRecieved>("PoseRecieved");
    factory.registerNodeType<behaviour_tree::DriveX>("DriveX");
    factory.registerNodeType<behaviour_tree::RotateX>("RotateX");

    // Get the path to our package's share directory (where resources are installed)
    std::string pkg_share_directory = ament_index_cpp::get_package_share_directory("behaviour_tree");
    
    // Construct the full path to our XML file that defines the behavior tree structure
    std::string xml_file = pkg_share_directory + "/bt_xml/bt_tree.xml";

    // Create a blackboard - this is the shared memory/data store for all BT nodes
    // Nodes can read/write data to the blackboard through input/output ports
    auto blackboard = BT::Blackboard::create();
    
    // Store our ROS2 node in the blackboard so all BT nodes can access it
    // Each BT node will retrieve this in their constructor via:
    // node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node")
    blackboard->set("node", node);

    // Load and create the behavior tree from the XML file
    // This instantiates all the nodes according to the tree structure in the XML
    // The blackboard is passed so nodes can access shared data (like the ROS node)
    auto tree = factory.createTreeFromFile(xml_file, blackboard);
    
    // Create a logger that prints tree execution to console
    // This shows which nodes are being ticked and their return status (helpful for debugging)
    BT::StdCoutLogger logger_cout(tree);

    // Create a ROS2 rate limiter to control loop frequency
    // 10 Hz means the behavior tree will be ticked 10 times per second
    rclcpp::Rate rate(10);

    // Main execution loop - runs until ROS2 is shut down (Ctrl+C)
    while (rclcpp::ok())
    {
        // Tick the behavior tree - this executes one iteration of the tree
        // The tree evaluates its nodes starting from the root and returns a status
        tree.tickRoot();
        
        // Process any pending ROS2 callbacks (subscriptions, timers, action callbacks, etc.)
        // This allows our action clients to receive responses and BT nodes to get messages
        // spin_some() processes callbacks without blocking
        rclcpp::spin_some(node);
        
        // Sleep to maintain the 10 Hz loop rate
        // This prevents the CPU from being maxed out and gives time for ROS2 messages
        rate.sleep();
    }
}