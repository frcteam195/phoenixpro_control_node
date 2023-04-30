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
#include "phoenixpro_control_node/CombinedCANcoderStatus.hpp"
#include "phoenixpro_control_node/CombinedPigeon2Status.hpp"

#define NODE_NAME "phoenixpro_control_node"

using namespace ctre::phoenixpro;

static constexpr units::frequency::hertz_t UPDATE_FREQUENCY = 100_Hz;

class LocalNode : public ParameterizedNode
{
public:
    LocalNode() : ParameterizedNode(NODE_NAME)
    {
        motor_status_publisher = this->create_publisher<ck_ros2_base_msgs_node::msg::MotorStatusArray>("/MotorStatus", 10);
        motor_control_subscriber = this->create_subscription<std_msgs::msg::String>("/MotorControl", 1, std::bind(&LocalNode::control_msg_callback, this, std::placeholders::_1));

        m_pigeon2 = new hardware::Pigeon2(0, Parameters.canivore_name.as_string());
        m_combined_pigeon2_status = new CombinedPigeon2Status(m_pigeon2, UPDATE_FREQUENCY);
        create_combined_wait_vector();
        
        create_motor(1);
        create_motor(2);

        create_cancoder(21);
        create_cancoder(22);
        create_cancoder(23);
        create_cancoder(24);

        status_listener_thread = std::thread(std::bind(&LocalNode::status_receiver_thread, this));
    }

    ~LocalNode()
    {
        for (auto p : motor_map)
        {
            if (p.second)
            {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
                delete p.second;
#pragma GCC diagnostic pop
            }
        }

        for (auto p : motor_status_map)
        {
            if (p.second)
            {
                delete p.second;
            }
        }

        if (m_pigeon2)
        {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
                delete m_pigeon2;
#pragma GCC diagnostic pop
        }

        if (m_combined_pigeon2_status)
        {
            delete m_combined_pigeon2_status;
        }
    }

private:
    std::thread status_listener_thread;

    hardware::Pigeon2* m_pigeon2;
    CombinedPigeon2Status* m_combined_pigeon2_status;

    std::map<int, hardware::TalonFX*> motor_map;
    std::map<int, CombinedMotorStatus*> motor_status_map;

    std::map<int, hardware::CANcoder*> cancoder_map;
    std::map<int, CombinedCANcoderStatus*> cancoder_status_map;

    std::recursive_mutex status_mutex;

    rclcpp::Publisher<ck_ros2_base_msgs_node::msg::MotorStatusArray>::SharedPtr motor_status_publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motor_control_subscriber;

    std::vector<BaseStatusSignalValue*> combined_status_signal_vector;

    void create_motor(int id)
    {
        if (!motor_map.count(id))
        {
            motor_map[id] = new hardware::TalonFX(id, Parameters.canivore_name.as_string());
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

    void create_cancoder(int id)
    {
        if (!cancoder_map.count(id))
        {
            cancoder_map[id] = new hardware::CANcoder(id, Parameters.canivore_name.as_string());
        }
        {
            std::scoped_lock<std::recursive_mutex> lock(status_mutex);
            if (!cancoder_status_map.count(id))
            {
                cancoder_status_map[id] = new CombinedCANcoderStatus(cancoder_map[id], UPDATE_FREQUENCY);
            }
        }

        create_combined_wait_vector();
    }

    void delete_cancoder(int id)
    {
        {
            std::scoped_lock<std::recursive_mutex> lock(status_mutex);
            if (cancoder_status_map.count(id))
            {
                CombinedCANcoderStatus* c = cancoder_status_map[id];
                cancoder_status_map.erase(id);
                if (c)
                {
                    delete c;
                }
            }
        }

        if (cancoder_map.count(id))
        {
            hardware::CANcoder* cancoder = cancoder_map[id];
            cancoder_map.erase(id);
            if (cancoder)
            {
                //TODO: Make sure this doesn't cause horrible issues based on compiler warning
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
                delete cancoder;
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

        for (auto p : cancoder_status_map)
        {
            const std::vector<BaseStatusSignalValue*>& input_vector = p.second->get_status_signal_vector();
            combined_status_signal_vector.insert(combined_status_signal_vector.end(), input_vector.begin(), input_vector.end());
        }

        if (m_pigeon2 && m_combined_pigeon2_status)
        {
            const std::vector<BaseStatusSignalValue*>& pigeon2_input_vector = m_combined_pigeon2_status->get_status_signal_vector();
            combined_status_signal_vector.insert(combined_status_signal_vector.end(), pigeon2_input_vector.begin(), pigeon2_input_vector.end());
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
                    m.id = p.second->get_status(MotorStatusType::DEVICE_ID);
                    m.sensor_position = p.second->get_status(MotorStatusType::POSITION);
                    m.sensor_velocity = p.second->get_status(MotorStatusType::VELOCITY);
                    m.bus_voltage = p.second->get_status(MotorStatusType::SUPPLY_VOLTAGE);
                    m.bus_current = p.second->get_status(MotorStatusType::SUPPLY_CURRENT);
                    m.stator_current = p.second->get_status(MotorStatusType::STATOR_CURRENT);
                    m.forward_limit_closed = p.second->get_status(MotorStatusType::FORWARD_LIMIT);
                    m.reverse_limit_closed = p.second->get_status(MotorStatusType::REVERSE_LIMIT);
                    m.control_mode = p.second->get_status(MotorStatusType::CONTROL_MODE);
                    m.commanded_output = p.second->get_status(MotorStatusType::CLOSED_LOOP_TARGET);
                    m.raw_output_percent = p.second->get_status(MotorStatusType::OUTPUT_DUTY_CYCLE);
                    motor_status_msg.motors.push_back(m);
                }

                motor_status_publisher->publish(motor_status_msg);
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
