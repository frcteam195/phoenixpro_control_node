#include "phoenixpro_control_node/phoenixpro_control_node.hpp"
#include "rclcpp/rclcpp.hpp"
#define UNIT_LIB_DISABLE_FMT
#define UNIT_LIB_ENABLE_IOSTREAM
#include "ctre/phoenixpro/TalonFX.hpp"
#include <cstdio>
#include <string>
#include <iostream>

#define NODE_NAME "phoenixpro_control_node"
static constexpr char CAN_NAME[] = "jetsoncanivore1";


using namespace ctre::phoenixpro;


class LocalNode : public rclcpp::Node
{
public:
    LocalNode() : rclcpp::Node(NODE_NAME)
    {

    }
private:
    hardware::TalonFX leftMaster{1, CAN_NAME};
    hardware::TalonFX rightMaster{2, CAN_NAME};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
