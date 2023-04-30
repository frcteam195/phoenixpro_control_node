#pragma once

#include "phoenixpro_control_node/SynchronizedStatus.hpp"

using namespace ctre::phoenixpro;

enum class Pigeon2StatusType
{
    DEVICE_ID,
    YAW,
    PITCH,
    ROLL
};

class CombinedPigeon2Status : public SynchronizedStatus
{
public:
    CombinedPigeon2Status(hardware::Pigeon2* pigeon2, units::frequency::hertz_t update_frequency)
    {
        m_pigeon2 = pigeon2;
        device_id = pigeon2->GetDeviceID();

        pigeon2->GetYaw().SetUpdateFrequency(update_frequency);
        pigeon2->GetPitch().SetUpdateFrequency(update_frequency);
        pigeon2->GetRoll().SetUpdateFrequency(update_frequency);

        status_values[Pigeon2StatusType::YAW] = &pigeon2->GetYaw();
        status_values[Pigeon2StatusType::PITCH] = &pigeon2->GetPitch();
        status_values[Pigeon2StatusType::ROLL] = &pigeon2->GetRoll();

        add_status_signal_to_wait_vector(status_values[Pigeon2StatusType::YAW]);
        add_status_signal_to_wait_vector(status_values[Pigeon2StatusType::PITCH]);
        add_status_signal_to_wait_vector(status_values[Pigeon2StatusType::ROLL]);
    }

    double get_status(Pigeon2StatusType status_type)
    {
        try
        {
            switch (status_type)
            {
                case Pigeon2StatusType::DEVICE_ID:
                {
                    return device_id;
                }
                case Pigeon2StatusType::YAW:
                {
                    StatusSignalValue<units::angle::degree_t>* sensor_yaw = dynamic_cast<StatusSignalValue<units::angle::degree_t>*>(status_values[status_type]);
                    return sensor_yaw->GetValue().value();
                }
                case Pigeon2StatusType::PITCH:
                {
                    StatusSignalValue<units::angle::degree_t>* sensor_pitch = dynamic_cast<StatusSignalValue<units::angle::degree_t>*>(status_values[status_type]);
                    return sensor_pitch->GetValue().value();
                }
                case Pigeon2StatusType::ROLL:
                {
                    StatusSignalValue<units::angle::degree_t>* sensor_roll = dynamic_cast<StatusSignalValue<units::angle::degree_t>*>(status_values[status_type]);
                    return sensor_roll->GetValue().value();
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
    std::map<Pigeon2StatusType, BaseStatusSignalValue*> status_values;

    int device_id = -1;
    hardware::Pigeon2* m_pigeon2;
};