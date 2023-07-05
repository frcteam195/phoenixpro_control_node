#include "phoenixpro_control_node/phoenixpro_control_node.hpp"
#ifdef HELLO_WORLD
#include <cstdio>
#include <string>
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#ifndef UNIT_LIB_DISABLE_FMT
    #define UNIT_LIB_DISABLE_FMT
#endif
#ifndef UNIT_LIB_ENABLE_IOSTREAM
    #define UNIT_LIB_ENABLE_IOSTREAM
#endif
#include "ctre/phoenixpro/TalonFX.hpp"
#include "ctre/phoenixpro/CANcoder.hpp"
#include "ctre/phoenixpro/Pigeon2.hpp"

#define NODE_NAME "phoenixpro_control_node"

using namespace ctre::phoenixpro;

class LocalNode : public ParameterizedNode
{
public:
    LocalNode() : ParameterizedNode(NODE_NAME), motor1(1), motor2(4)
    {
        test_thread = std::thread(std::bind(&LocalNode::run_thread, this));
    }

private:
    void run_thread()
    {
        controls::DutyCycleOut dco(0);
        dco.WithOutput(0);
        while (rclcpp::ok())
        {
            motor1.SetControl(dco);
        }
    }

    std::thread test_thread;
    ctre::phoenixpro::hardware::TalonFX motor1;
    ctre::phoenixpro::hardware::TalonFX motor2;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#endif