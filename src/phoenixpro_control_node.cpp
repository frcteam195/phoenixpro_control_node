#include "phoenixpro_control_node/phoenixpro_control_node.hpp"
#include <cstdio>
#include <string>
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ck_ros2_base_msgs_node/msg/motor_status.hpp"
#include "ck_ros2_base_msgs_node/msg/motor_status_array.hpp"

#ifndef UNIT_LIB_DISABLE_FMT
    #define UNIT_LIB_DISABLE_FMT
#endif
#ifndef UNIT_LIB_ENABLE_IOSTREAM
    #define UNIT_LIB_ENABLE_IOSTREAM
#endif
#include "ctre/phoenixpro/TalonFX.hpp"

#include "phoenixpro_control_node/CombinedMotorStatus.hpp"

#define NODE_NAME "phoenixpro_control_node"

using namespace ctre::phoenixpro;

static constexpr units::frequency::hertz_t UPDATE_FREQUENCY = 100_Hz;

class LocalNode : public ParameterizedNode
{
public:
    LocalNode() : ParameterizedNode(NODE_NAME)
    {
        status_publisher = this->create_publisher<ck_ros2_base_msgs_node::msg::MotorStatusArray>("/MotorStatus", 10);
        control_subscriber = this->create_subscription<std_msgs::msg::String>("/MotorControl", 1, std::bind(&LocalNode::control_msg_callback, this, std::placeholders::_1));

        create_motor(1);
        create_motor(2);

        status_listener_thread = std::thread(std::bind(&LocalNode::status_receiver_thread, this));
    }

private:
    std::thread status_listener_thread;

    std::map<int, hardware::TalonFX*> motor_map;
    std::map<int, CombinedMotorStatus*> motor_status_map;

    std::recursive_mutex status_mutex;

    rclcpp::Publisher<ck_ros2_base_msgs_node::msg::MotorStatusArray>::SharedPtr status_publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_subscriber;

    std::vector<BaseStatusSignalValue*> combined_status_signal_vector;

    void create_motor(int id)
    {
        if (!motor_map.count(id))
        {
            motor_map[id] = new hardware::TalonFX(id, params[Parameters::canivore_name].as_string());
        }
        {
            std::scoped_lock<std::recursive_mutex> lock(status_mutex);
            if (!motor_status_map.count(id))
            {
                motor_status_map[id] = new CombinedMotorStatus(motor_map[id], UPDATE_FREQUENCY);
            }
        }

        create_combined_wait_vector();
    }

    void delete_motor(int id)
    {
        {
            std::scoped_lock<std::recursive_mutex> lock(status_mutex);
            if (motor_status_map.count(id))
            {
                CombinedMotorStatus* c = motor_status_map[id];
                motor_status_map.erase(id);
                if (c)
                {
                    delete c;
                }
            }
        }

        if (motor_map.count(id))
        {
            hardware::TalonFX* tfx = motor_map[id];
            motor_map.erase(id);
            if (tfx)
            {
                //TODO: Make sure this doesn't cause horrible issues based on compiler warning
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
                delete tfx;
#pragma GCC diagnostic pop
            }
        }

        create_combined_wait_vector();
    }


    void create_combined_wait_vector()
    {
        std::scoped_lock<std::recursive_mutex> lock(status_mutex);
        combined_status_signal_vector.clear();
        for (auto p : motor_status_map)
        {
            const std::vector<BaseStatusSignalValue*>& input_vector = p.second->get_status_signal_vector();
            combined_status_signal_vector.insert(combined_status_signal_vector.end(), input_vector.begin(), input_vector.end());
        }
    }

    void control_msg_callback(const std_msgs::msg::String::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    void status_receiver_thread()
    {
        rclcpp::Rate default_rate(std::chrono::milliseconds(10));
        std::vector<BaseStatusSignalValue*> local_combined_status_wait_vector;
        while (rclcpp::ok())
        {
            {
                std::scoped_lock<std::recursive_mutex> lock(status_mutex);
                local_combined_status_wait_vector = combined_status_signal_vector;

                ck_ros2_base_msgs_node::msg::MotorStatusArray motor_status_msg;
                for(auto p : motor_status_map)
                {
                    ck_ros2_base_msgs_node::msg::MotorStatus m;
                    m.id = p.second->get_status(StatusType::DEVICE_ID);
                    m.sensor_position = p.second->get_status(StatusType::POSITION);
                    m.sensor_velocity = p.second->get_status(StatusType::VELOCITY);
                    m.bus_voltage = p.second->get_status(StatusType::SUPPLY_VOLTAGE);
                    m.bus_current = p.second->get_status(StatusType::SUPPLY_CURRENT);
                    m.stator_current = p.second->get_status(StatusType::STATOR_CURRENT);
                    m.forward_limit_closed = p.second->get_status(StatusType::FORWARD_LIMIT);
                    m.reverse_limit_closed = p.second->get_status(StatusType::REVERSE_LIMIT);
                    m.control_mode = p.second->get_status(StatusType::CONTROL_MODE);
                    m.commanded_output = p.second->get_status(StatusType::CLOSED_LOOP_TARGET);
                    m.raw_output_percent = p.second->get_status(StatusType::OUTPUT_DUTY_CYCLE);
                    motor_status_msg.motors.push_back(m);
                }

                status_publisher->publish(motor_status_msg);
            }

            if (local_combined_status_wait_vector.size() > 0)
            {
                BaseStatusSignalValue::WaitForAll(15_ms, local_combined_status_wait_vector);
            }
            else
            {
                default_rate.sleep();
            }
        }
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
