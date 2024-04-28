// this node has its class fully defined in the cpp file
// for larger nodes, the class can be classically split in header / source


// include any thing required - do not forget to use the .hpp extension for ROS 2 files
#include <rclcpp/rclcpp.hpp>
#include <baxter_core_msgs/msg/joint_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <algorithm>

// access time units such as 100ms
using namespace std::chrono_literals;
using std::placeholders::_1;

// some shortcuts for message classes
using sensor_msgs::msg::JointState;
using baxter_core_msgs::msg::JointCommand;

// a useful function to get the index of a string in a vector of strings
// returns the size of the vector if not found
inline size_t findIndex(const std::string &name, const std::vector<std::string> & names)
{
  const auto elem = std::find(names.begin(), names.end(), name);
  return std::distance(names.begin(), elem);
}

namespace lab2_mirror
{

class MirrorNode : public rclcpp::Node
{
public:
  MirrorNode(rclcpp::NodeOptions options) : Node("mirror", options)
  {
    // init whatever is needed for your node
    // these suffixes may be useful
    // const std::vector<std::string> suffixes = {"_s0", "_s1", "_e0", "_e1", "_w0", "_w1", "_w2"};

    // joint mapping
    joint_mapping_ = {
      {"right_s0", "left_s0"},
      {"right_s1", "left_s1"},
      {"right_e0", "left_e0"},
      {"right_e1", "left_e1"},
      {"right_w0", "left_w0"},
      {"right_w1", "left_w1"},
      {"right_w2", "left_w2"}
    };

    // init command message
    // command_msg_.mode = JointCommand::POSITION_MODE;
    command_msg_.mode = 1;

    // init subscriber
    joint_state_subscription_ = this->create_subscription<JointState>("robot/joint_states", 10, std::bind(&MirrorNode::joint_state_callback, this, _1));

    // init publisher
    command_publisher_ = this->create_publisher<JointCommand>("robot/limb/left/joint_command", 10);
  }

private:
    std::map<std::string, std::string> joint_mapping_;
    JointCommand command_msg_;
    rclcpp::Subscription<JointState>::SharedPtr joint_state_subscription_;
    rclcpp::Publisher<JointCommand>::SharedPtr command_publisher_;

    void joint_state_callback(const JointState::SharedPtr msg)
    {
        // check receaving signal
        // RCLCPP_INFO(this->get_logger(), "Received joint state message");

        // clear previous command messages
        command_msg_.names.clear();
        command_msg_.command.clear();

        for (const auto& mapping : joint_mapping_) {
            size_t index = findIndex(mapping.first, msg->name);   // mapping.first is right joints
          
            if (index != msg->name.size()) {
                command_msg_.names.push_back(mapping.second);     // mapping.second is left joints
                double position = msg->position[index];
              
                // specific joint(s0, e0, w0, w2) have to change the sign to be mirroring
                if (mapping.first == "right_s0" || mapping.first == "right_e0" || mapping.first == "right_w0" || mapping.first == "right_w2") {
                        position = -position;
                }
              
                command_msg_.command.push_back(position);
              
            }
        }
        if (!command_msg_.names.empty()) {
                command_publisher_->publish(command_msg_);
        }
    }
};

}

// boilerplate main function

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lab2_mirror::MirrorNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}



/* Keep Memo

<Rub Command>
colcon build --packages-select lab2_mirror
source install/setup.bash
ros2 run lab2_mirror mirror_node

---------------------------------------------------------

ros2 interface show sensor_msgs/msg/JointState
std_msgs/Header header
        builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
        string frame_id

string[] name
float64[] position
float64[] velocity
float64[] effort

---------------------------------------------------------

ros2 interface show baxter_core_msgs/msg/JointCommand
int32 mode
float64[] command
string[] names

int32 POSITION_MODE =1
int32 VELOCITY_MODE =2
int32 TORQUE_MODE =3
int32 RAW_POSITION_MODE =4

 */
