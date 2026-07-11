#pragma once
#include <cmath>
#include <vector>

#include "chassis/chassis_config.hpp"
#include "gimbal/gimbal_config.hpp"
#include "gimbal/gimbal_temp.hpp"
#include "pid_controller.hpp"
#include "shoot_config.hpp"
#include "types.hpp"

namespace Config
{
    const Chassis ::ChassisConfig chassis_config ={
        .wheels_config ={
            Hardware::DJIMotorConfig{3508, "can1", 1, 0.075},
            Hardware::DJIMotorConfig{3508, "can1", 2, 0.075},
            Hardware::DJIMotorConfig{3508, "can1", 3, 0.075},
            Hardware::DJIMotorConfig{3508, "can1", 4, 0.075}
        }
        .chassis_follow_gimbal_pid_config = {
            .kp =           2.0f,
            .ki =           0.0f,
            .kd =           10.0f,
            .max_out =      6.0f,
            .max_iout =     0.2f,
        },
        .wheel_speed_pid_config = {
            .kp =           15000.0f,
            .ki =           10.0f,
            .kd =           0.0f,
            .max_out =      14000.0f,
            .max_iout =     2000.0f,
        },
        .ControlTime = 2,
				.follow_dir = -1,
    }
     /** CHASSIS **/
    const typename Pid::PidConfig M3508_SPEED_PID_CONFIG{
        15000.0f,  // KP
        10.0f,     // KI
        0.0f,      // KD
        14000.0f,  // MAX_OUT
        2000.0f,   // MAX_IOUT
    };

    const typename Pid::PidConfig CHASSIS_FOLLOW_GIMBAL_PID_CONFIG{
        4.0f,   // KP
        0.0f,   // KI
        10.0f,  // KD
        6.0f,   // MAX_OUT
        0.2f,   // MAX_IOUT
    };

    // m3508 rmp change to chassis speed,
    // m3508转化成底盘速度(m/s)的比例，
    constexpr fp32 CHASSIS_MOTOR_RPM_TO_VECTOR_SEN = 0.000415809748903494517209f;
    constexpr fp32 SHOOT_MOTOR_RPM_TO_SPEED = 0.00290888208665721596153948461415f;
    constexpr fp32 M6020_ECD_TO_RAD = 2.f * M_PIf / 8192.f;
    constexpr fp32 M9025_ECD_TO_RAD = 2.f * M_PIf / 65535.f;
    constexpr fp32 RPM_TO_RAD_S = 2.f * M_PIf / 60.f;
    constexpr fp32 temp = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN / (RPM_TO_RAD_S / 19.f);
    constexpr fp32 CHASSIS_CONTROL_FREQUENCE = 500.0f;
#define STAND
#ifdef STAND
    constexpr fp32 GIMBAL1_YAW_OFFSET_ECD = 3459;
    constexpr fp32 GIMBAL1_PITCH_OFFSET_ECD = 2194;

    constexpr fp32 GIMBAL2_YAW_OFFSET_ECD = 3366;
    constexpr fp32 GIMBAL2_PITCH_OFFSET_ECD = 3985;

    constexpr fp32 GIMBAL3_YAW_OFFSET_ECD = 48041;
    constexpr fp32 GIMBAL3_PITCH_OFFSET_ECD = 3985;
#else
    constexpr fp32 GIMBAL_YAW_OFFSET_ECD = 5424;
    constexpr fp32 GIMBAL_PITCH_OFFSET_ECD = 618;
#endif

    constexpr uint32_t GIMBAL_INIT_STOP_TIME = 2000;
    constexpr fp32 GIMBAL_INIT_EXP = 0.1f;

    constexpr fp32 FRICTION_MAX_SPEED = 2.5f;
    constexpr fp32 FRICTION_ADD_SPEED = 1.0f;
    constexpr fp32 CONTINUE_TRIGGER_SPEED = 6.f;

    constexpr uint32_t CHASSIS_CONTROL_TIME = 2;
    constexpr uint32_t GIMBAL_CONTROL_TIME = 1;
    constexpr uint32_t SHOOT_CONTROL_TIME = 1;

    constexpr uint32_t DEFAULT_OFFLINE_TIME = 1000;

}  // namespace Config