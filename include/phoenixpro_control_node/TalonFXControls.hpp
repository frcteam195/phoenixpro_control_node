#pragma once

#ifndef UNIT_LIB_DISABLE_FMT
    #define UNIT_LIB_DISABLE_FMT
#endif
#ifndef UNIT_LIB_ENABLE_IOSTREAM
    #define UNIT_LIB_ENABLE_IOSTREAM
#endif
#include "ctre/phoenixpro/TalonFX.hpp"

using namespace ctre::phoenixpro::controls;

class TalonFXControls
{
public:
    DutyCycleOut duty_cycle{0};
    TorqueCurrentFOC torque_current_foc{units::current::ampere_t(0)};
    VoltageOut voltage{units::voltage::volt_t(0)};
    PositionDutyCycle position_duty_cycle{units::angle::turn_t(0)};
    PositionTorqueCurrentFOC position_torque_current_foc{units::angle::turn_t(0)};
    PositionVoltage position_voltage{units::angle::turn_t(0)};
    VelocityDutyCycle velocity_duty_cycle{units::angular_velocity::turns_per_second_t(0)};
    VelocityTorqueCurrentFOC velocity_torque_current_foc{units::angular_velocity::turns_per_second_t(0)};
    VelocityVoltage velocity_voltage{units::angular_velocity::turns_per_second_t(0)};
    MotionMagicDutyCycle motionmagic_duty_cycle{units::angle::turn_t(0)};
    MotionMagicTorqueCurrentFOC motionmagic_torque_current_foc{units::angle::turn_t(0)};
    MotionMagicVoltage motionmagic_voltage{units::angle::turn_t(0)};
    NeutralOut neutral{};
    StaticBrake static_brake{};
    CoastOut coast{};
    // Follower follower{};
};