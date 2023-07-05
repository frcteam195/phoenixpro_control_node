#pragma once

#ifndef UNIT_LIB_DISABLE_FMT
    #define UNIT_LIB_DISABLE_FMT
#endif
#ifndef UNIT_LIB_ENABLE_IOSTREAM
    #define UNIT_LIB_ENABLE_IOSTREAM
#endif
#include "ctre/phoenixpro/TalonFX.hpp"
#include "phoenixpro_control_node/CombinedMotorStatus.hpp"
#include "phoenixpro_control_node/TalonFXControls.hpp"
#include <string>
using namespace ctre::phoenixpro;

class ROSTalonFX
{
public:
    ROSTalonFX(int motor_id, units::frequency::hertz_t update_frequency, std::string canbus_name)
    {
        (void)update_frequency;
        motor = new hardware::TalonFX(motor_id, canbus_name);
        motor_status = new CombinedMotorStatus(motor, update_frequency);
        motor_configuration = new configs::TalonFXConfiguration();
        motor_control = new TalonFXControls();
    }

    ~ROSTalonFX()
    {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
        if (motor)
        {
            delete motor;
        }
        if (motor_status)
        {
            delete motor_status;
        }
        if (motor_configuration)
        {
            delete motor_configuration;
        }
        if (motor_control)
        {
            delete motor_control;
        }
#pragma GCC diagnostic pop
    }

    bool motor_is_follower = false;
    uint8_t master_motor_id = 0;

    hardware::TalonFX* motor = nullptr;
    CombinedMotorStatus* motor_status = nullptr;
    configs::TalonFXConfiguration* motor_configuration = nullptr;
    configs::TalonFXConfiguration prev_motor_configuration;
    TalonFXControls* motor_control = nullptr;
};