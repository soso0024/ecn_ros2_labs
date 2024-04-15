// this node has its class fully defined in the cpp file
// for larger nodes, the class can be classically split in header / source


// include any thing required - do not forget to use the .hpp extension for ROS 2 files
#include <rclcpp/rclcpp.hpp>
#include <baxter_core_msgs/msg/joint_command.hpp>
#include <baxter_core_msgs/srv/solve_position_ik.hpp>
#include <algorithm>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "ik_client.h"

using namespace std::chrono_literals;
using baxter_core_msgs::msg::JointCommand;
using baxter_core_msgs::srv::SolvePositionIK;

namespace lab3_puppet
{

class PuppetNode : public rclcpp::Node
{
public:
  PuppetNode(rclcpp::NodeOptions options) : Node("puppet", options)
  {
    // init whatever is needed for your node

    // init command message for left arm
    left_arm_command_ = JointCommand();

    // init publisher to left arm command
    command_publisher_ = this->create_publisher<JointCommand>("left_arm_command", 10);

    // init timer - the function publishCommand() should called with the given rate
    timer_ = this->create_wall_timer(1000ms, std::bind(&PuppetNode::publishCommand, this));

    // IK service wrapper into IKNode
    ik_node.init("ik_node","/ExternalTools/left/PositionKinematicsNode/IKService");
  }
  
private:

  // declare member variables for command publisher and timer

  ServiceNodeSync<SolvePositionIK> ik_node;

  // TF 2 stuff
  tf2_ros::Buffer tf_buffer{get_clock()};                // stores all previous elementary transforms in a tree
  tf2_ros::TransformListener tf_listener{tf_buffer};   // subscribes to /tf

  void publishCommand()
  {
    // check if the transform from base to left_gripper_desired is available
    if(tf_buffer.canTransform("left_gripper_desired", "base", tf2::TimePointZero, tf2::durationFromSec(1.0)))
    {
      // get this transform with tf_buffer.lookupTransform("base", "left_gripper_desired", ...
	  auto transform = tf_buffer.lookupTransform("base", "left_gripper_desired", tf2::TimePointZero);

      // build service request SolvePositionIK::Request from obtained transform
      SolvePositionIK::Request req;
	  req.pose.header.frame_id = "base";
	  req.pose.pose.position.x = transform.transform.translation.x;
	  req.pose.pose.position.y = transform.transform.translation.y;
	  req.pose.pose.position.z = transform.transform.translation.z;
	  req.pose.pose.orientation = transform.transform.rotation;
      
      // call service and get response
      if (ik_node.call(req, res) && res.is_valid)
	  {
		  // call to IK was successfull, check if the solution is valid
		  JointCommand command;
		  command.mode = JointCommand::POSITION_MODE;
		  for (auto& joint : res.joints)
		  {
			command.names.push_back(joint.name);
			command.command.push_back(joint.position);
		  }
		  // copy response data to joint command and publish to left arm
		  command_publisher_->publish(command);
	   }
    }
  }
};
}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lab3_puppet::PuppetNode)
