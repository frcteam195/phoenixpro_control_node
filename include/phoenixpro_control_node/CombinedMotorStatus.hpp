#pragma once

#include "phoenixpro_control_node/SynchronizedStatus.hpp"

using namespace ctre::phoenixpro;

enum class MotorStatusType
{
    DEVICE_ID,
    POSITION,
    VELOCITY,
    SUPPLY_VOLTAGE,
    SUPPLY_CURRENT,
    STATOR_CURRENT,
    FORWARD_LIMIT,
    REVERSE_LIMIT,
    CONTROL_MODE,
    CLOSED_LOOP_TARGET,
    OUTPUT_DUTY_CYCLE,
};

class CombinedMotorStatus : public SynchronizedStatus
{
public:
    CombinedMotorStatus(hardware::TalonFX* talon_fx, units::frequency::hertz_t update_frequency)
    {
        m_talonfx = talon_fx;
        device_id = talon_fx->GetDeviceID();

        talon_fx->GetPosition().SetUpdateFrequency(update_frequency);
        talon_fx->GetVelocity().SetUpdateFrequency(update_frequency);
        talon_fx->GetSupplyVoltage().SetUpdateFrequency(update_frequency);
        talon_fx->GetSupplyCurrent().SetUpdateFrequency(update_frequency);
        talon_fx->GetStatorCurrent().SetUpdateFrequency(update_frequency);
        talon_fx->GetForwardLimit().SetUpdateFrequency(update_frequency);
        talon_fx->GetReverseLimit().SetUpdateFrequency(update_frequency);
        talon_fx->GetControlMode().SetUpdateFrequency(update_frequency);
        talon_fx->GetClosedLoopReference().SetUpdateFrequency(update_frequency);
        talon_fx->GetDutyCycle().SetUpdateFrequency(update_frequency);

        status_values[MotorStatusType::POSITION] = &talon_fx->GetPosition();
        status_values[MotorStatusType::VELOCITY] = &talon_fx->GetVelocity();
        status_values[MotorStatusType::SUPPLY_VOLTAGE] = &talon_fx->GetSupplyVoltage();
        status_values[MotorStatusType::SUPPLY_CURRENT] = &talon_fx->GetSupplyCurrent();
        status_values[MotorStatusType::STATOR_CURRENT] = &talon_fx->GetStatorCurrent();
        status_values[MotorStatusType::FORWARD_LIMIT] = &talon_fx->GetForwardLimit();
        status_values[MotorStatusType::REVERSE_LIMIT] = &talon_fx->GetReverseLimit();
        status_values[MotorStatusType::CONTROL_MODE] = &talon_fx->GetControlMode();
        status_values[MotorStatusType::CLOSED_LOOP_TARGET] = &talon_fx->GetClosedLoopReference();
        status_values[MotorStatusType::OUTPUT_DUTY_CYCLE] = &talon_fx->GetDutyCycle();

        add_status_signal_to_wait_vector(status_values[MotorStatusType::POSITION]);
        add_status_signal_to_wait_vector(status_values[MotorStatusType::VELOCITY]);
        add_status_signal_to_wait_vector(status_values[MotorStatusType::SUPPLY_VOLTAGE]);
        add_status_signal_to_wait_vector(status_values[MotorStatusType::SUPPLY_CURRENT]);
        add_status_signal_to_wait_vector(status_values[MotorStatusType::STATOR_CURRENT]);
        add_status_signal_to_wait_vector(status_values[MotorStatusType::FORWARD_LIMIT]);
        add_status_signal_to_wait_vector(status_values[MotorStatusType::REVERSE_LIMIT]);
        add_status_signal_to_wait_vector(status_values[MotorStatusType::CONTROL_MODE]);
        add_status_signal_to_wait_vector(status_values[MotorStatusType::CLOSED_LOOP_TARGET]);
        add_status_signal_to_wait_vector(status_values[MotorStatusType::OUTPUT_DUTY_CYCLE]);
    }

    double get_status(MotorStatusType status_type)
    {
        try
        {
            switch (status_type)
            {
                case MotorStatusType::DEVICE_ID:
                {
                    return device_id;
                }
                case MotorStatusType::POSITION:
                {
                    StatusSignalValue<units::angle::turn_t>* sensor_position = dynamic_cast<StatusSignalValue<units::angle::turn_t>*>(status_values[status_type]);
                    return sensor_position->GetValue().value();
                }
                case MotorStatusType::VELOCITY:
                {
                    StatusSignalValue<units::angular_velocity::turns_per_second_t>* sensor_velocity = dynamic_cast<StatusSignalValue<units::angular_velocity::turns_per_second_t>*>(status_values[status_type]);
                    return sensor_velocity->GetValue().value();
                }
                case MotorStatusType::SUPPLY_VOLTAGE:
                {
                    StatusSignalValue<units::voltage::volt_t>* supply_voltage = dynamic_cast<StatusSignalValue<units::voltage::volt_t>*>(status_values[status_type]);
                    return supply_voltage->GetValue().value();
                }
                case MotorStatusType::SUPPLY_CURRENT:
                {
                    StatusSignalValue<units::current::ampere_t>* supply_current = dynamic_cast<StatusSignalValue<units::current::ampere_t>*>(status_values[status_type]);
                    return supply_current->GetValue().value();
                }
                case MotorStatusType::STATOR_CURRENT:
                {
                    StatusSignalValue<units::current::ampere_t>* stator_current = dynamic_cast<StatusSignalValue<units::current::ampere_t>*>(status_values[status_type]);
                    return stator_current->GetValue().value();
                }
                case MotorStatusType::FORWARD_LIMIT:
                {
                    StatusSignalValue<signals::ForwardLimitValue>* forward_limit = dynamic_cast<StatusSignalValue<signals::ForwardLimitValue>*>(status_values[status_type]);
                    return forward_limit->GetValue().value == signals::ForwardLimitValue::ClosedToGround ? 1 : 0;
                }
                case MotorStatusType::REVERSE_LIMIT:
                {
                    StatusSignalValue<signals::ReverseLimitValue>* reverse_limit = dynamic_cast<StatusSignalValue<signals::ReverseLimitValue>*>(status_values[status_type]);
                    return reverse_limit->GetValue().value == signals::ReverseLimitValue::ClosedToGround ? 1 : 0;
                }
                case MotorStatusType::CONTROL_MODE:
                {
                    StatusSignalValue<signals::ControlModeValue>* control_mode = dynamic_cast<StatusSignalValue<signals::ControlModeValue>*>(status_values[status_type]);
                    return (int)(control_mode->GetValue().value);
                }
                case MotorStatusType::CLOSED_LOOP_TARGET:
                {
                    StatusSignalValue<double>* closed_loop_target = dynamic_cast<StatusSignalValue<double>*>(status_values[status_type]);
                    return closed_loop_target->GetValue();
                }
                case MotorStatusType::OUTPUT_DUTY_CYCLE:
                {
                    StatusSignalValue<units::dimensionless::scalar_t>* output_duty_cycle = dynamic_cast<StatusSignalValue<units::dimensionless::scalar_t>*>(status_values[status_type]);
                    return output_duty_cycle->GetValue().value();
                }
            }
        }
        catch (std::exception& e)
        {
            std::cout << "Error occurred casting status type: " << (int)status_type << std::endl;
        }
        return 0;
    }

private:
    std::map<MotorStatusType, BaseStatusSignalValue*> status_values;

    int device_id = -1;
    hardware::TalonFX* m_talonfx;
    // StatusSignalValue<units::angle::turn_t> sensor_position;
    // StatusSignalValue<units::angular_velocity::turns_per_second_t> sensor_velocity;
    // StatusSignalValue<units::voltage::volt_t> bus_voltage;
    // StatusSignalValue<units::current::ampere_t> bus_current;
    // StatusSignalValue<units::current::ampere_t> stator_current;
    // StatusSignalValue<signals::ForwardLimitValue> forward_limit;
    // StatusSignalValue<signals::ReverseLimitValue> reverse_limit;
    // StatusSignalValue<signals::ControlModeValue> control_mode;
    // StatusSignalValue<double> closed_loop_target;
    // StatusSignalValue<units::dimensionless::scalar_t> raw_output_duty_cycle;
};