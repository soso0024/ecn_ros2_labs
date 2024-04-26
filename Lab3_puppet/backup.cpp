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

    // init publisher to left arm command
    publisher_ = this->create_publisher<JointCommand>("robot", 10);

    // init timer - the function publishCommand() should called with the given rate
    timer_ = this->create_wall_timer(500ms, std::bind(&PuppetNode::publishCommand, this));

    // IK service wrapper into IKNode
    ik_node.init("ik_node","/ExternalTools/left/PositionKinematicsNode/IKService");
  }

private:

  // declare member variables for command publisher and timer
  rclcpp::Publisher<JointCommand>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  ServiceNodeSync<SolvePositionIK> ik_node;

  // TF 2 stuff
  tf2_ros::Buffer tf_buffer{get_clock()};                // stores all previous elementary transforms in a tree
  tf2_ros::TransformListener tf_listener{tf_buffer};     // subscribes to /tf

  void publishCommand()
  {
    geometry_msgs::msg::TransformStamped transform;
    baxter_core_msgs::msg::JointCommand command_msg;

    // check if the transform from base to left_gripper_desired is available
    if(tf_buffer.canTransform("left_gripper_desired", "base", tf2::TimePointZero, tf2::durationFromSec(1.0)))
    {
      // get this transform with tf_buffer.lookupTransform("base", "left_gripper_desired", ...
      transform = tf_buffer.lookupTransform("base", "left_gripper_desired", tf2::TimePointZero);

      // build service request SolvePositionIK::Request from obtained transform
      SolvePositionIK::Request req;
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.pose.position.x = transform.transform.translation.x;
      pose_stamped.pose.position.y = transform.transform.translation.y;
      pose_stamped.pose.position.z = transform.transform.translation.z;
      pose_stamped.pose.orientation.x = transform.transform.rotation.x;
      pose_stamped.pose.orientation.y = transform.transform.rotation.y;
      pose_stamped.pose.orientation.z = transform.transform.rotation.z;

      req.pose_stamp.push_back(pose_stamped);

      // call service and get response
      if(SolvePositionIK::Response res; ik_node.call(req, res))
      {
        // call to IK was successfull, check if the solution is valid

        // copy response data to joint command and publish to left arm
        publisher_->publish(command_msg);

      }
    }
  }
};
}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lab3_puppet::PuppetNode)
