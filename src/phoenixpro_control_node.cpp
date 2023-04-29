#include "phoenixpro_control_node/phoenixpro_control_node.hpp"
#include "rclcpp/rclcpp.hpp"
#define UNIT_LIB_DISABLE_FMT
#define UNIT_LIB_ENABLE_IOSTREAM
#include "ctre/phoenixpro/TalonFX.hpp"
#include <cstdio>
#include <string>
#include <iostream>
#include "std_msgs/msg/string.hpp"

#define NODE_NAME "phoenixpro_control_node"
static constexpr char CAN_NAME[] = "jetsoncanivore1";

using namespace ctre::phoenixpro;
using std::placeholders::_1;

class LocalNode : public rclcpp::Node
{
public:
    LocalNode() : rclcpp::Node(NODE_NAME)
    {
        status_publisher = this->create_publisher<std_msgs::msg::String>("/MotorStatus", 10);
        control_subscriber = this->create_subscription<std_msgs::msg::String>("/MotorControl", 1, std::bind(&LocalNode::control_msg_callback, this, _1));
    }
private:
    hardware::TalonFX leftMaster{1, CAN_NAME};
    hardware::TalonFX rightMaster{2, CAN_NAME};

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_subscriber;

    void control_msg_callback(const std_msgs::msg::String::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
