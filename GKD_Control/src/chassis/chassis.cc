#include "chassis/chassis.hpp"

#include <array>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <thread>
#include "logger.hpp"
#include "socket_interface.hpp"
#include "robot_type_config.hpp"
#include "user_lib.hpp"
#include "utils.hpp"

namespace Chassis
{
    // 轮电机反馈丢失判定时间：C620 约 1 kHz 反馈，40 ms 约等于连续丢 40 帧
    constexpr uint32_t WHEEL_OFFLINE_TIME_MS = 40;

    Chassis::Chassis(const ChassisConfig &config)
        : config(config),
          motors(config.wheels_config.begin(), config.wheels_config.end()),
          power_manager(motors, Power::Division::HERO) {
    }

    void Chassis::init(const std::shared_ptr<Robot::Robot_set> &robot) {
        robot_set = robot;

        power_manager.init(robot);
        power_manager.setMode(1);

        chassis_angle_pid = Pid::PidRad(
                                config.chassis_follow_gimbal_pid_config,
                                MUXDEF(
                                    CONFIG_SENTRY,
                                    robot_set->gimbal_sentry_yaw_reletive,
                                    robot_set->gimbalT_1_yaw_reletive)) >>
                            Pid::Invert(config.follow_dir);

        for (auto &motor : motors) {
            motor.set_offline_time(WHEEL_OFFLINE_TIME_MS);
            motor.setCtrl(Pid::PidPosition(
                config.wheel_speed_pid_config, motor.data_.output_linear_velocity));
            motor.enable();
        }

        for (int i = 0; i < 4; i++) {
            wheels_pid[i] = Pid::PidPosition(
                config.wheel_speed_pid_config, motors[i].data_.output_linear_velocity);
        }
    }

    [[noreturn]] void Chassis::task() {
        std::jthread power_daemon(&Power::Manager::powerDaemon, &power_manager);
        while (true) { 
            if (!robot_set->referee_info.game_robot_status_data.mains_power_chassis_output) {
                for (auto &motor : motors) {
                    motor.set_zero();
                }

                for (int i = 0; i < 4; i++) {
                    wheels_pid[i].clean();
                    wheel_speed[i] = 0.f;
                }
                chassis_angle_pid.clean();

                vx_set = 0.f;
                vy_set = 0.f;
                wz_set = 0.f;
                last_wz_direction = 0.f;
                robot_set->spin_state = false;

                robot_set->chassis_recover_mode = true;

                UserLib::sleep_ms(config.ControlTime);
                continue;
            }
            decomposition_speed();
            // LOG_INFO("chassis.wheel_speed: %f, %f, %f, %f\n", wheel_speed[0], wheel_speed[1], wheel_speed[2], wheel_speed[3]);
            if (robot_set->mode == Types::ROBOT_MODE::ROBOT_NO_FORCE) {
                for (auto &motor : motors) {
                    motor.set_zero();
                }
                for (int i = 0; i < 4; ++i) {
                    wheels_pid[i].clean();
                }
                chassis_angle_pid.clean();
                robot_set->spin_state = false;
            } else {
                fp32 max_speed = 0.f;
                for (int i = 0; i < 4; i++) {
                    // 离线轮的目标不参与限速统计，避免它的错误目标拖慢健康轮
                    if (motors[i].offline()) {
                        continue;
                    }
                    max_speed = std::max(max_speed, fabsf(wheel_speed[i]));
                }
                //TODO 增加加速度限制
                if (max_speed > max_wheel_speed) {
                    fp32 speed_rate = max_wheel_speed / max_speed;
                    for (int i = 0; i < 4; i++) {
                        wheel_speed[i] *= speed_rate;
                    }
                }

                for (int i = 0; i < 4; i++) {
                    if (motors[i].offline()) {
                        // 离线期间每周期清积分，恢复时不会带着旧积分反冲
                        wheels_pid[i].clean();
                    } else {
                        wheels_pid[i].set(wheel_speed[i]);
                    }
                }

                robot_set->spin_state = robot_set->wz_set < 0.1 ? false : true;
                // LOG_INFO("spin?: %d\n", robot_set->spin_state);

                // Power Limit
                for (int i = 0; i < 4; ++i) {
                    if (motors[i].offline()) {
                        // 离线电机不参与本轮功率分配，避免用过期反馈占用功率预算
                        objs[i].curAv = 0.f;
                        objs[i].pidOutput = 0.f;
                        objs[i].setAv = 0.f;
                        objs[i].pidMaxOutput = 0.f;
                        continue;
                    }
                    // 统一到输出轴（轮端）量纲，和 k0/k1/k2 保持一致
                    objs[i].curAv = motors[i].data_.output_angular_velocity;
                    objs[i].pidOutput = wheels_pid[i].out;
                    objs[i].setAv = wheel_speed[i] / motors[i].data_.radius;
                    objs[i].pidMaxOutput = 14000;
                }
                static Power::PowerObj *pObjs[4] = { &objs[0], &objs[1], &objs[2], &objs[3] };
                std::array<float, 4> cmd_power = power_manager.getControlledOutput(pObjs);

                //logger
                // for (int i = 0; i < 4; ++i) {
                //    logger.push_value("chassis." + std::to_string(i), cmd_power[i]);
                //    logger.push_console_message("<h1>111</h1>");
                // }

                for (int i = 0; i < 4; ++i) {
                    // 离线电机不再输出，避免使用过期反馈继续驱动
                    motors[i].give_current = motors[i].offline() ? 0.f : cmd_power[i];
                    //LOG_INFO("i:%d, plan:%f, fact:%f\n", i, cmd_power[i], robot_set->super_cap_info.chassisPower);
                }
            }
            UserLib::sleep_ms(config.ControlTime);
        }
    }

    void Chassis::decomposition_speed() {
        if (robot_set->mode != Types::ROBOT_MODE::ROBOT_NO_FORCE) {
            fp32 sin_yaw, cos_yaw;
            MUXDEF(
                CONFIG_SENTRY,
                sincosf(robot_set->gimbal_sentry_yaw_reletive, &sin_yaw, &cos_yaw),
                sincosf(robot_set->gimbalT_1_yaw_reletive, &sin_yaw, &cos_yaw));
            vx_set = cos_yaw * robot_set->vx_set + sin_yaw * robot_set->vy_set;
            vy_set = -sin_yaw * robot_set->vx_set + cos_yaw * robot_set->vy_set;

            if (robot_set->wz_set == 0.f) {  
                bool recover_handled = false;
                if (robot_set->chassis_recover_mode) {
                    fp32 current_angle = MUXDEF(
                        CONFIG_SENTRY,
                        robot_set->gimbal_sentry_yaw_reletive,
                        robot_set->gimbalT_1_yaw_reletive);

                    if (fabs(current_angle) > 0.1f) {
                        chassis_angle_pid.set(0.f);
                        if (chassis_angle_pid.out > 0.f) {
                            wz_set = 1.f;
                        } else if (chassis_angle_pid.out < 0.f) {
                            wz_set = -1.f;
                        } else {
                            wz_set = 0.f;
                        }
                        recover_handled = true;
                    } else {
                        robot_set->chassis_recover_mode = false;
                        last_wz_direction = 0.f;
                    }
                }

                if (!recover_handled) {
                    if (last_wz_direction != 0.f) {  
                        fp32 current_angle = MUXDEF(  
                            CONFIG_SENTRY,  
                            robot_set->gimbal_sentry_yaw_reletive,  
                            robot_set->gimbalT_1_yaw_reletive);  
                        if (fabs(current_angle) > 0.1f && fabs(current_angle) < 0.6f) {  
                            wz_set = last_wz_direction - 0.5;    
                        }else if(fabs(current_angle) > 0.6f){
                            wz_set = last_wz_direction;
                        } 
                        else {  
                            chassis_angle_pid.set(0.f);  
                            wz_set = chassis_angle_pid.out;  
                            last_wz_direction = 0.f;   
                        }                     
                    } else {  
                        chassis_angle_pid.set(0.f);  
                        wz_set = chassis_angle_pid.out;  
                    }
                }
            } else {  
            wz_set = robot_set->wz_set;  
            robot_set->chassis_recover_mode = false;
            last_wz_direction = wz_set > 0 ? 1.0f : -1.0f; 
            }
    }

        wheel_speed[0] = -vx_set - vy_set + wz_set;
        wheel_speed[1] = -vx_set + vy_set + wz_set;
        wheel_speed[2] = vx_set - vy_set + wz_set;
        wheel_speed[3] = vx_set + vy_set + wz_set;
        
    }
}  // namespace Chassis
