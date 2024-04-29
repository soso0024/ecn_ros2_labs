// this node has its class fully defined in the cpp file
// for larger nodes, the class can be classically split in header / source


// include any thing required - do not forget to use the .hpp extension for ROS 2 files
#include <rclcpp/rclcpp.hpp>
#include <baxter_core_msgs/msg/joint_command.hpp>
#include <baxter_core_msgs/srv/solve_position_ik.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <algorithm>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "ik_client.h"

using namespace std::chrono_literals;
using baxter_core_msgs::msg::JointCommand;
using baxter_core_msgs::srv::SolvePositionIK;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::PoseStamped;

namespace lab3_puppet
{

class PuppetNode : public rclcpp::Node
{
public:
  PuppetNode(rclcpp::NodeOptions options) : Node("puppet", options)
  {
    // init publisher to left arm command
    publisher_ = this->create_publisher<JointCommand>("robot/limb/left/joint_command", 10);

    // init timer - the function publishCommand() should called with the given rate
    timer_ = this->create_wall_timer(50ms, std::bind(&PuppetNode::publishCommand, this));

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
    TransformStamped transform;
    JointCommand command_msg;

    // check if the transform from base to left_gripper_desired is available
    if(tf_buffer.canTransform("left_gripper_desired", "base", tf2::TimePointZero, tf2::durationFromSec(1.0)))
    {
      // get this transform with tf_buffer.lookupTransform("base", "left_gripper_desired", ...
      transform = tf_buffer.lookupTransform("base", "left_gripper_desired", tf2::TimePointZero);

      // build service request SolvePositionIK::Request from obtained transform
      SolvePositionIK::Request req;
      PoseStamped pose_stamped;

      pose_stamped.pose.position.x = transform.transform.translation.x;
      pose_stamped.pose.position.y = transform.transform.translation.y;
      pose_stamped.pose.position.z = transform.transform.translation.z;
      pose_stamped.pose.orientation.x = transform.transform.rotation.x;
      pose_stamped.pose.orientation.y = transform.transform.rotation.y;
      pose_stamped.pose.orientation.z = transform.transform.rotation.z;

      req.pose_stamp.push_back(pose_stamped);
      command_msg.mode = 1;
      command_msg.set__names({"left_s0", "left_s1","left_e0","left_e1","left_w0","left_w1","left_w2"});
      command_msg.command.resize(command_msg.names.size(),0.0);

      // call service and get response
      if(SolvePositionIK::Response res; ik_node.call(req, res))
      {
        // call to IK was successfull, check if the solution is valid
          if(res.is_valid[0]){
              std::copy(res.joints[0].position.begin(), res.joints[0].position.end(), command_msg.command.begin());
//               Insted of Copy
//               command_msg.mode = 1;
//               command_msg.names = res.joints[0].name;
//               command_msg.command = res.joints[0].positions;
              publisher_->publish(command_msg);
          }
      }
    }
  }
};
}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lab3_puppet::PuppetNode)



/*
<Run Command>
After Simulation

ros2 run tf2_ros static_transform_publisher 0 0 0.1 0 3.14 0 right_gripper left_gripper_desired

ros2 run lab3_puppet puppet

------------------------------------------------------------------------------------------------------

[ROS2@baxter] ~/ros2$ ros2 interface show sensor_msgs/msg/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints.
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state.
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty.
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.

std_msgs/Header header
        builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
        string frame_id

string[] name
float64[] position
float64[] velocity
float64[] effort



[ROS2@baxter] ~/ros2$ ros2 interface show baxter_core_msgs/srv/SolvePositionIK
# Endpoint Pose(s) to request Inverse-Kinematics joint solutions for.
geometry_msgs/PoseStamped[] pose_stamp
        std_msgs/Header header
                builtin_interfaces/Time stamp
                        int32 sec
                        uint32 nanosec
                string frame_id
        Pose pose
                Point position
                        float64 x
                        float64 y
                        float64 z
                Quaternion orientation
                        float64 x 0
                        float64 y 0
                        float64 z 0
                        float64 w 1

# (optional) Joint Angle Seed(s) for IK solver.
# * specify a JointState seed for each pose_stamp, using name[] and position[]
# * empty arrays or a non-default seed_mode will cause user seed to not be used
sensor_msgs/JointState[] seed_angles
        #
        #
        #
        #
        std_msgs/Header header
                builtin_interfaces/Time stamp
                        int32 sec
                        uint32 nanosec
                string frame_id
        string[] name
        float64[] position
        float64[] velocity
        float64[] effort

# Seed Type Mode
# * default (SEED_AUTO) mode: iterate through seed types until first valid
#                             solution is found
# * setting any other mode:   try only that seed type
uint8 SEED_AUTO = 0
uint8 SEED_USER = 1
uint8 SEED_CURRENT = 2
uint8 SEED_NS_MAP = 3

uint8 seed_mode
---
# joints[i]      == joint angle solution for each pose_state[i]
sensor_msgs/JointState[] joints
        #
        #
        #
        #
        std_msgs/Header header
                builtin_interfaces/Time stamp
                        int32 sec
                        uint32 nanosec
                string frame_id
        string[] name
        float64[] position
        float64[] velocity
        float64[] effort

# NOTE: isValid will be deprecated by result_type in future versions
bool[] is_valid

# result_type[i] == seed type used to find valid solution, joints[i];
# otherwise,     == RESULT_INVALID (no valid solution found).
uint8 RESULT_INVALID = 0
uint8[] result_type



[ROS2@baxter] ~/ros2$ ros2 interface show geometry_msgs/msg/TransformStamped
# This expresses a transform from coordinate frame header.frame_id
# to the coordinate frame child_frame_id at the time of header.stamp
#
# This message is mostly used by the
# <a href="https://index.ros.org/p/tf2/">tf2</a> package.
# See its documentation for more information.
#
# The child_frame_id is necessary in addition to the frame_id
# in the Header to communicate the full reference for the transform
# in a self contained message.

# The frame id in the header is used as the reference frame of this transform.
std_msgs/Header header
        builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
        string frame_id

# The frame id of the child frame to which this transform points.
string child_frame_id

# Translation and rotation in 3-dimensions of child_frame_id from header.frame_id.
Transform transform
        Vector3 translation
                float64 x
                float64 y
                float64 z
        Quaternion rotation
                float64 x 0
                float64 y 0
                float64 z 0
                float64 w 1

*/

