#pragma once

#include "phoenixpro_control_node/SynchronizedStatus.hpp"

using namespace ctre::phoenixpro;

enum class CANcoderStatusType
{
    DEVICE_ID,
    ABSOLUTE_POSITION,
    MAGNET_HEALTH
};

class CombinedCANcoderStatus : public SynchronizedStatus
{
public:
    CombinedCANcoderStatus(hardware::CANcoder* cancoder, units::frequency::hertz_t update_frequency)
    {
        m_cancoder = cancoder;
        device_id = cancoder->GetDeviceID();

        cancoder->GetAbsolutePosition().SetUpdateFrequency(update_frequency);
        cancoder->GetMagnetHealth().SetUpdateFrequency(update_frequency);

        status_values[CANcoderStatusType::ABSOLUTE_POSITION] = &cancoder->GetAbsolutePosition();
        status_values[CANcoderStatusType::MAGNET_HEALTH] = &cancoder->GetMagnetHealth();

        add_status_signal_to_wait_vector(status_values[CANcoderStatusType::ABSOLUTE_POSITION]);
        add_status_signal_to_wait_vector(status_values[CANcoderStatusType::MAGNET_HEALTH]);
    }

    double get_status(CANcoderStatusType status_type)
    {
        try
        {
            switch (status_type)
            {
                case CANcoderStatusType::DEVICE_ID:
                {
                    return device_id;
                }
                case CANcoderStatusType::ABSOLUTE_POSITION:
                {
                    StatusSignalValue<units::angle::turn_t>* sensor_abs_position = dynamic_cast<StatusSignalValue<units::angle::turn_t>*>(status_values[status_type]);
                    return sensor_abs_position->GetValue().value();
                }
                case CANcoderStatusType::MAGNET_HEALTH:
                {
                    StatusSignalValue<signals::MagnetHealthValue>* sensor_abs_position = dynamic_cast<StatusSignalValue<signals::MagnetHealthValue>*>(status_values[status_type]);
                    return sensor_abs_position->GetValue().value;
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
    std::map<CANcoderStatusType, BaseStatusSignalValue*> status_values;

    int device_id = -1;
    hardware::CANcoder* m_cancoder;
};