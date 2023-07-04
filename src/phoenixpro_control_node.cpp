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
#include "ck_ros2_base_msgs_node/msg/motor_configuration.hpp"
#include "ck_ros2_base_msgs_node/msg/motor_configuration_array.hpp"
#include "ck_ros2_base_msgs_node/msg/motor_control.hpp"
#include "ck_ros2_base_msgs_node/msg/motor_control_array.hpp"
#include "ck_ros2_base_msgs_node/msg/motor_control_mode_type.hpp"
#include "ck_ros2_base_msgs_node/msg/motor_control_feed_forward_type.hpp"

#include "ck_utilities_ros2_node/node_handle.hpp"

rclcpp::Node::SharedPtr node_handle;

#ifndef UNIT_LIB_DISABLE_FMT
    #define UNIT_LIB_DISABLE_FMT
#endif
#ifndef UNIT_LIB_ENABLE_IOSTREAM
    #define UNIT_LIB_ENABLE_IOSTREAM
#endif
#include "ctre/phoenixpro/TalonFX.hpp"
#include "ctre/phoenixpro/CANcoder.hpp"
#include "ctre/phoenixpro/Pigeon2.hpp"

#include "phoenixpro_control_node/CombinedMotorStatus.hpp"
#include "phoenixpro_control_node/CombinedCANcoderStatus.hpp"
#include "phoenixpro_control_node/CombinedPigeon2Status.hpp"

#include "phoenixpro_control_node/ROSTalonFX.hpp"

#define NODE_NAME "phoenixpro_control_node"

using namespace ctre::phoenixpro;

static constexpr units::frequency::hertz_t UPDATE_FREQUENCY = 100_Hz;

class LocalNode : public ParameterizedNode
{
public:
    LocalNode() : ParameterizedNode(NODE_NAME)
    {
        motor_status_publisher = this->create_publisher<ck_ros2_base_msgs_node::msg::MotorStatusArray>("/MotorStatus", 10);
        motor_control_subscriber = this->create_subscription<ck_ros2_base_msgs_node::msg::MotorControlArray>("/MotorControl", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(), std::bind(&LocalNode::control_msg_callback, this, std::placeholders::_1));
        motor_config_subscriber = this->create_subscription<ck_ros2_base_msgs_node::msg::MotorConfigurationArray>("/MotorConfiguration", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(), std::bind(&LocalNode::config_msg_callback, this, std::placeholders::_1));

        m_pigeon2 = new hardware::Pigeon2(0, Parameters.canivore_name);
        m_combined_pigeon2_status = new CombinedPigeon2Status(m_pigeon2, UPDATE_FREQUENCY);
        create_combined_wait_vector();

        // create_cancoder(21);
        // create_cancoder(22);
        // create_cancoder(23);
        // create_cancoder(24);

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

    std::map<int, ROSTalonFX*> motor_map;

    std::map<int, hardware::CANcoder*> cancoder_map;
    std::map<int, CombinedCANcoderStatus*> cancoder_status_map;

    std::recursive_mutex status_mutex;

    rclcpp::Publisher<ck_ros2_base_msgs_node::msg::MotorStatusArray>::SharedPtr motor_status_publisher;
    rclcpp::Subscription<ck_ros2_base_msgs_node::msg::MotorControlArray>::SharedPtr motor_control_subscriber;
    rclcpp::Subscription<ck_ros2_base_msgs_node::msg::MotorConfigurationArray>::SharedPtr motor_config_subscriber;

    std::vector<BaseStatusSignalValue*> combined_status_signal_vector;

    void create_motor(int id)
    {
        if (!motor_map.count(id))
        {
            motor_map[id] = new ROSTalonFX(id, UPDATE_FREQUENCY, Parameters.canivore_name);
        }

        create_combined_wait_vector();
    }

    void delete_motor(int id)
    {
        if (motor_map.count(id))
        {
            auto tfx = motor_map[id];
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
            cancoder_map[id] = new hardware::CANcoder(id, Parameters.canivore_name);
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
        for (auto p : motor_map)
        {
            const std::vector<BaseStatusSignalValue*>& input_vector = p.second->motor_status->get_status_signal_vector();
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

    void control_msg_callback(const ck_ros2_base_msgs_node::msg::MotorControlArray::SharedPtr motor_control_array)
    {
        for (auto m : motor_control_array->motors)
        {
            if (!motor_map.count(m.id))
            {
                return;
            }

            auto control = motor_map[m.id]->motor_control;
            auto motor = motor_map[m.id]->motor;

            if (motor_map[m.id]->motor_is_follower)
            {
                motor->SetControl(StrictFollower(motor_map[m.id]->master_motor_id));
                return;
            }

            using ck_ros2_base_msgs_node::msg::MotorControlModeType;
            using ck_ros2_base_msgs_node::msg::MotorControlFeedForwardType;
            switch(m.control_mode)
            {
                case MotorControlModeType::DUTY_CYCLE:
                {
                    motor->SetControl(control->duty_cycle.WithEnableFOC(true).WithOutput(m.setpoint));
                    break;
                }
                case MotorControlModeType::TORQUE_CURRENT:
                {
                    motor->SetControl(control->torque_current_foc.WithOutput(units::current::ampere_t(m.setpoint)));
                    break;
                }
                case MotorControlModeType::VOLTAGE:
                {
                    motor->SetControl(control->voltage.WithOutput(units::voltage::volt_t(m.setpoint)));
                    break;
                }
                case MotorControlModeType::POSITION:
                {
                    switch (m.feed_forward_type)
                    {
                        case MotorControlFeedForwardType::NONE:
                        {
                            motor->SetControl(control->position_duty_cycle.WithEnableFOC(true).WithFeedForward(0).WithSlot(m.gain_slot).WithPosition(units::angle::turn_t(m.setpoint)));
                            break;
                        }
                        case MotorControlFeedForwardType::DUTY_CYCLE:
                        {
                            motor->SetControl(control->position_duty_cycle.WithEnableFOC(true).WithFeedForward(m.feed_forward).WithSlot(m.gain_slot).WithPosition(units::angle::turn_t(m.setpoint)));
                            break;
                        }
                        case MotorControlFeedForwardType::TORQUE_CURRENT:
                        {
                            motor->SetControl(control->position_torque_current_foc.WithFeedForward(units::current::ampere_t(m.feed_forward)).WithSlot(m.gain_slot).WithPosition(units::angle::turn_t(m.setpoint)));
                            break;
                        }
                        case MotorControlFeedForwardType::VOLTAGE:
                        {
                            motor->SetControl(control->position_voltage.WithEnableFOC(true).WithFeedForward(units::voltage::volt_t(m.feed_forward)).WithSlot(m.gain_slot).WithPosition(units::angle::turn_t(m.setpoint)));
                            break;
                        }
                        default:
                        {
                            RCLCPP_ERROR(this->get_logger(), "Default case reached. This is a problem.");
                            break;
                        }
                    }
                    break;
                }
                case MotorControlModeType::VELOCITY:
                {
                    switch (m.feed_forward_type)
                    {
                        case MotorControlFeedForwardType::NONE:
                        {
                            motor->SetControl(control->velocity_duty_cycle.WithEnableFOC(true).WithFeedForward(0).WithSlot(m.gain_slot).WithVelocity(units::angular_velocity::turns_per_second_t(m.setpoint)));
                            break;
                        }
                        case MotorControlFeedForwardType::DUTY_CYCLE:
                        {
                            motor->SetControl(control->velocity_duty_cycle.WithEnableFOC(true).WithFeedForward(m.feed_forward).WithSlot(m.gain_slot).WithVelocity(units::angular_velocity::turns_per_second_t(m.setpoint)));
                            break;
                        }
                        case MotorControlFeedForwardType::TORQUE_CURRENT:
                        {
                            motor->SetControl(control->velocity_torque_current_foc.WithFeedForward(units::current::ampere_t(m.feed_forward)).WithSlot(m.gain_slot).WithVelocity(units::angular_velocity::turns_per_second_t(m.setpoint)));
                            break;
                        }
                        case MotorControlFeedForwardType::VOLTAGE:
                        {
                            motor->SetControl(control->velocity_voltage.WithEnableFOC(true).WithFeedForward(units::voltage::volt_t(m.feed_forward)).WithSlot(m.gain_slot).WithVelocity(units::angular_velocity::turns_per_second_t(m.setpoint)));
                            break;
                        }
                        default:
                        {
                            RCLCPP_ERROR(this->get_logger(), "Default case reached. This is a problem.");
                            break;
                        }
                    }
                    break;
                }
                case MotorControlModeType::MOTION_MAGIC:
                {
                    switch (m.feed_forward_type)
                    {
                        case MotorControlFeedForwardType::NONE:
                        {
                            motor->SetControl(control->motionmagic_duty_cycle.WithEnableFOC(true).WithFeedForward(0).WithSlot(m.gain_slot).WithPosition(units::angle::turn_t(m.setpoint)));
                            break;
                        }
                        case MotorControlFeedForwardType::DUTY_CYCLE:
                        {
                            motor->SetControl(control->motionmagic_duty_cycle.WithEnableFOC(true).WithFeedForward(m.feed_forward).WithSlot(m.gain_slot).WithPosition(units::angle::turn_t(m.setpoint)));
                            break;
                        }
                        case MotorControlFeedForwardType::TORQUE_CURRENT:
                        {
                            //TODO: Fix units once CTRE API is fixed
                            // motor->SetControl(control->motionmagic_torque_current_foc.WithFeedForward(units::current::ampere_t(m.feed_forward)).WithSlot(m.gain_slot).WithPosition(units::angle::turn_t(m.setpoint)));
                            motor->SetControl(control->motionmagic_torque_current_foc.WithFeedForward(units::dimensionless::scalar_t(m.feed_forward)).WithSlot(m.gain_slot).WithPosition(units::angle::turn_t(m.setpoint)));
                            break;
                        }
                        case MotorControlFeedForwardType::VOLTAGE:
                        {
                            motor->SetControl(control->motionmagic_voltage.WithEnableFOC(true).WithFeedForward(units::voltage::volt_t(m.feed_forward)).WithSlot(m.gain_slot).WithPosition(units::angle::turn_t(m.setpoint)));
                            break;
                        }
                        default:
                        {
                            RCLCPP_ERROR(this->get_logger(), "Default case reached. This is a problem.");
                            break;
                        }
                    }
                    break;
                }
                case MotorControlModeType::NEUTRAL_OUT:
                {
                    motor->SetControl(control->neutral);
                    break;
                }
                case MotorControlModeType::STATIC_BRAKE:
                {
                    motor->SetControl(control->static_brake);
                    break;
                }
                case MotorControlModeType::COAST_OUT:
                {
                    motor->SetControl(control->coast);
                    break;
                }
                default:
                {
                    break;
                }
            }
        }
    }

    void config_msg_callback(const ck_ros2_base_msgs_node::msg::MotorConfigurationArray::SharedPtr motor_config_array)
    {
        for (auto m : motor_config_array->motors)
        {
            if (!motor_map.count(m.id))
            {
                create_motor(m.id);
            }

            if (m.master_id > 0)
            {
                motor_map[m.id]->motor_is_follower = true;
                motor_map[m.id]->master_motor_id = m.master_id;
            }
            
            auto config = motor_map[m.id]->motor_configuration;
            config->MotorOutput.Inverted = m.invert ? signals::InvertedValue::Clockwise_Positive : signals::InvertedValue::CounterClockwise_Positive;
            config->MotorOutput.NeutralMode = m.brake_neutral ? signals::NeutralModeValue::Brake : signals::NeutralModeValue::Coast;
            
            auto kp_arr = m.k_p;
            auto ki_arr = m.k_i;
            auto kd_arr = m.k_d;
            auto kv_arr = m.k_v;
            auto ks_arr = m.k_s;

            size_t check_size = 0;
            if (kp_arr.size() > check_size && ki_arr.size() > check_size && kd_arr.size() > check_size && kv_arr.size() > check_size && ks_arr.size() > check_size)
            {
                config->Slot0.kP = kp_arr[0];
                config->Slot0.kI = ki_arr[0];
                config->Slot0.kD = kd_arr[0];
                config->Slot0.kV = kv_arr[0];
                config->Slot0.kS = ks_arr[0];
            }

            check_size = 1;
            if (kp_arr.size() > check_size && ki_arr.size() > check_size && kd_arr.size() > check_size && kv_arr.size() > check_size && ks_arr.size() > check_size)
            {
                config->Slot1.kP = kp_arr[1];
                config->Slot1.kI = ki_arr[1];
                config->Slot1.kD = kd_arr[1];
                config->Slot1.kV = kv_arr[1];
                config->Slot1.kS = ks_arr[1];
            }

            check_size = 2;
            if (kp_arr.size() > check_size && ki_arr.size() > check_size && kd_arr.size() > check_size && kv_arr.size() > check_size && ks_arr.size() > check_size)
            {
                config->Slot2.kP = kp_arr[2];
                config->Slot2.kI = ki_arr[2];
                config->Slot2.kD = kd_arr[2];
                config->Slot2.kV = kv_arr[2];
                config->Slot2.kS = ks_arr[2];
            }

            config->CurrentLimits.StatorCurrentLimitEnable = m.enable_stator_current_limit;
            config->CurrentLimits.StatorCurrentLimit = m.stator_current_limit;
            config->CurrentLimits.SupplyCurrentLimitEnable = m.enable_supply_current_limit;
            config->CurrentLimits.SupplyCurrentLimit = m.supply_current_limit;
            config->CurrentLimits.SupplyCurrentThreshold = m.supply_current_threshold;
            config->CurrentLimits.SupplyTimeThreshold = m.supply_time_threshold;
            config->ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = m.duty_cycle_closed_loop_ramp_period;
            config->ClosedLoopRamps.TorqueClosedLoopRampPeriod = m.torque_current_closed_loop_ramp_period;
            config->ClosedLoopRamps.VoltageClosedLoopRampPeriod = m.voltage_closed_loop_ramp_period;
            config->OpenLoopRamps.DutyCycleOpenLoopRampPeriod = m.duty_cycle_open_loop_ramp_period;
            config->OpenLoopRamps.TorqueOpenLoopRampPeriod = m.torque_current_open_loop_ramp_period;
            config->OpenLoopRamps.VoltageOpenLoopRampPeriod = m.voltage_open_loop_ramp_period;
            config->HardwareLimitSwitch.ForwardLimitEnable = false;
            config->HardwareLimitSwitch.ReverseLimitEnable = false;
            config->SoftwareLimitSwitch.ForwardSoftLimitEnable = m.enable_forward_soft_limit;
            config->SoftwareLimitSwitch.ReverseSoftLimitEnable = m.enable_reverse_soft_limit;
            config->SoftwareLimitSwitch.ForwardSoftLimitThreshold = m.forward_soft_limit_threshold;
            config->SoftwareLimitSwitch.ReverseSoftLimitThreshold = m.reverse_soft_limit_threshold;
            config->MotionMagic.MotionMagicAcceleration = m.motion_magic_acceleration;
            config->MotionMagic.MotionMagicCruiseVelocity = m.motion_magic_cruise_velocity;
            config->MotionMagic.MotionMagicJerk = m.motion_magic_jerk;

            motor_map[m.id]->motor->GetConfigurator().Apply(*config);
        }
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
                for(auto p : motor_map)
                {
                    CombinedMotorStatus* motor_status_tfx = p.second->motor_status;
                    ck_ros2_base_msgs_node::msg::MotorStatus m;
                    m.id = motor_status_tfx->get_status(MotorStatusType::DEVICE_ID);
                    m.sensor_position = motor_status_tfx->get_status(MotorStatusType::POSITION);
                    m.sensor_velocity = motor_status_tfx->get_status(MotorStatusType::VELOCITY);
                    m.bus_voltage = motor_status_tfx->get_status(MotorStatusType::SUPPLY_VOLTAGE);
                    m.bus_current = motor_status_tfx->get_status(MotorStatusType::SUPPLY_CURRENT);
                    m.stator_current = motor_status_tfx->get_status(MotorStatusType::STATOR_CURRENT);
                    m.forward_limit_closed = motor_status_tfx->get_status(MotorStatusType::FORWARD_LIMIT);
                    m.reverse_limit_closed = motor_status_tfx->get_status(MotorStatusType::REVERSE_LIMIT);
                    m.control_mode = motor_status_tfx->get_status(MotorStatusType::CONTROL_MODE);
                    m.commanded_output = motor_status_tfx->get_status(MotorStatusType::CLOSED_LOOP_TARGET);
                    m.raw_output_percent = motor_status_tfx->get_status(MotorStatusType::OUTPUT_DUTY_CYCLE);
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
    node_handle = std::make_shared<LocalNode>();
    rclcpp::spin(node_handle);
    rclcpp::shutdown();
    return 0;
}
