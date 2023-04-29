#include "phoenixpro_control_node/phoenixpro_control_node.hpp"
#include "rclcpp/rclcpp.hpp"
#define UNIT_LIB_DISABLE_FMT
#define UNIT_LIB_ENABLE_IOSTREAM
#include "ctre/phoenixpro/TalonFX.hpp"
#include <cstdio>
#include <string>
#include <iostream>
#include <vector>
#include "std_msgs/msg/string.hpp"
#include "ck_ros2_base_msgs_node/msg/motor_status.hpp"
#include "ck_ros2_base_msgs_node/msg/motor_status_array.hpp"

#define NODE_NAME "phoenixpro_control_node"
static constexpr char CAN_NAME[] = "jetsoncanivore1";

using namespace ctre::phoenixpro;

class LocalNode : public rclcpp::Node
{
public:
    LocalNode() : rclcpp::Node(NODE_NAME)
    {
        params = this->get_parameters(param_names);

        status_publisher = this->create_publisher<ck_ros2_base_msgs_node::msg::MotorStatusArray>("/MotorStatus", 10);
        control_subscriber = this->create_subscription<std_msgs::msg::String>("/MotorControl", 1, std::bind(&LocalNode::control_msg_callback, this, std::placeholders::_1));
    }
private:
    hardware::TalonFX leftMaster{1, CAN_NAME};
    hardware::TalonFX rightMaster{2, CAN_NAME};

    rclcpp::Publisher<ck_ros2_base_msgs_node::msg::MotorStatusArray>::SharedPtr status_publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_subscriber;

    std::vector<std::string> param_names = {
        "my_str",
        "my_int",
        "my_double_array"
    };
    std::vector<rclcpp::Parameter> params;

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
