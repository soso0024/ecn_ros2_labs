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
  JointCommand left_arm_command_;   // Declare the JointCommand for the left arm
  rclcpp::Publisher<JointCommand>::SharedPtr command_publisher_;  // Declare the publisher
  rclcpp::TimerBase::SharedPtr timer_;  // Declare the timer

  // TF 2 and IK service declarations remain the same
  ServiceNodeSync<SolvePositionIK> ik_node;
  tf2_ros::Buffer tf_buffer{get_clock()};                // stores all previous elementary transforms in a tree
  tf2_ros::TransformListener tf_listener{tf_buffer};   // subscribes to /tf

   void publishCommand()
   {
    if(tf_buffer.canTransform("left_gripper_desired", "base", tf2::TimePointZero, tf2::durationFromSec(1.0)))
    {
        auto transform = tf_buffer.lookupTransform("base", "left_gripper_desired", tf2::TimePointZero);
        SolvePositionIK::Request req;
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "base";
        pose_stamped.header.stamp = rclcpp::Clock().now();
        pose_stamped.pose.position.x = transform.transform.translation.x;
        pose_stamped.pose.position.y = transform.transform.translation.y;
        pose_stamped.pose.position.z = transform.transform.translation.z;
        pose_stamped.pose.orientation.w = transform.transform.rotation.w;
        pose_stamped.pose.orientation.x = transform.transform.rotation.x;
        pose_stamped.pose.orientation.y = transform.transform.rotation.y;
        pose_stamped.pose.orientation.z = transform.transform.rotation.z;
        req.pose_stamp.push_back(pose_stamped);

        if (SolvePositionIK::Response res; ik_node.call(req, res))
        {
            JointCommand command;
            command.mode = JointCommand::POSITION_MODE;
            for (const auto& joint : res.joints)
            {
                for (const auto& name : joint.name)
                {
                    command.names.push_back(name);
                }
                for (const auto& position : joint.position)
                {
                    command.command.push_back(position);
                }
            }
            command_publisher_->publish(command);
        }
    }
   }
};
}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lab3_puppet::PuppetNode)
