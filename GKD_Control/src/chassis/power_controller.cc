#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

#include "logger.hpp"
#include "macro_helpers.hpp"
#include "pid_controller.hpp"
#include "power_controller.hpp"
#include "robot_type_config.hpp"
#include "utils.hpp"

namespace Power
{

    PowerStatus powerStatus;
    // 用户可配置功率下限（运行期会按模式/规则动态更新）
    static float MIN_MAXPOWER_CONFIGURED = 40.0f;

    // 浮点比较辅助：避免直接比较造成阈值抖动
    static inline bool floatEqual(float a, float b) {
        return fabs(a - b) < 1e-5f;
    }

    // 转速(rpm) -> 角速度(rad/s)
    static inline float rpm2av(float rpm) {
        return rpm * (float)M_PI / 30.0f;
    }

    // 角速度(rad/s) -> 转速(rpm)
    static inline float av2rpm(float av) {
        return av * 30.0f / (float)M_PI;
    }

    // 错误标志位操作（按位或）
    static inline void setErrorFlag(uint8_t &curFlag, Manager::ErrorFlags setFlag) {
        curFlag |= static_cast<uint8_t>(setFlag);
    }

    // 错误标志位清除（按位与非）
    static inline void clearErrorFlag(uint8_t &curFlag, Manager::ErrorFlags clearFlag) {
        curFlag &= (~static_cast<uint8_t>(clearFlag));
    }

    // 判断某错误位是否置位
    static inline bool isFlagged(uint8_t &curFlag, Manager::ErrorFlags flag) {
        return (curFlag & static_cast<uint8_t>(flag)) != 0;
    }

    Manager::Manager(
        std::deque<Hardware::DJIMotor> &motors_,
        const Division division_,
        RLSEnabled rlsEnabled_,
        const float k1_,
        const float k2_,
        const float k3_,
        const float lambda_)

        : rlsEnabled(Manager::RLSEnabled::Enable),
          error(0UL),
          motors(motors_),
          division(division_),
          powerBuff(0.0f),
          fullBuffSet(0.0f),
          baseBuffSet(0.0f),
          fullMaxPower(0.0f),
          baseMaxPower(0.0f),
          powerUpperLimit(0.0f),
          refereeMaxPower(0.0f),
          userConfiguredMaxPower(0.0f),
          callback(nullptr),
          k1(k1_),
          k2(k2_),
          k3(k3_),
          lastUpdateTick(0),
          rls(1e-5f, 0.99999f) {
        // k1/k2/k3 物理上均应为非负，否则模型失真
        configASSERT(k1_ >= 0);
        configASSERT(k2_ >= 0);
        configASSERT(k3_ >= 0);

        // RLS 初值：用传入的 k1/k2 作为起始参数
        float initParams[2] = { k1_, k2_ };
        rls.setParamVector(Math::Matrixf<2, 1>(initParams));
    }

    static bool isInitialized;
    uint8_t xPowerTaskStack[1024];

    /**
     * @implements
     */
    void Manager::setMaxPowerConfigured(float maxPower) {
        // 最终可用上限会被运行时上下限再次夹紧
        userConfiguredMaxPower = std::clamp(maxPower, MIN_MAXPOWER_CONFIGURED, powerUpperLimit);
    }

    void Manager::setMode(uint8_t mode) {
        // mode=1: 放开到上限; 其他: 收敛到下限
        setMaxPowerConfigured(mode == 1 ? powerUpperLimit : powerLowerLimit);
    }

    /**
     * @implements
     */


std::array<float, 4> Manager::getControlledOutput(PowerObj *objs[4]) {
    // 先全零，离线轮和异常分支都不会留下未初始化值
    std::array<float, 4> newTorqueCurrent{};

    // 3508 输出轴近似扭矩常数 (N·m/A)，与 curAv=output_angular_velocity 同侧
    float torqueConst = 0.3f;
    // k0: 电流指令 -> 输出轴扭矩(N·m)比例
    // 20/16384: C620 电流指令量程，-16384..16384 对应 -20A..+20A
    float k0 = torqueConst * 20.0f / 16384.0f;  // N·m per command unit

    // sumCmdPower: 四轮在“未限功率”下的总估算功率
    float sumCmdPower = 0.0f;
    // cmdPower[i]: 第 i 轮的估算功率
    std::array<float, 4> cmdPower;

    // sumError: 全轮速度误差和（只统计正功率轮）
    float sumError = 0.0f;
    std::array<float, 4> error;

    // 当前循环可用最大功率，来自用户设置与能量环上下界综合约束
    // fullMaxPower/baseMaxPower 的大小关系会随能量环输出变化，先排序避免 clamp 的 lo > hi
    const float powerLowerBound = std::min(fullMaxPower, baseMaxPower);
    const float powerUpperBound = std::max(fullMaxPower, baseMaxPower);
    float maxPower = std::clamp(userConfiguredMaxPower, powerLowerBound, powerUpperBound);

    // 离线轮的 k3/4 作为不可避免的固定损耗预留，但不作为可分权重需求
    float offlineFixedLoss = 0.0f;
    for (int i = 0; i < 4; ++i) {
        if (motors[i].offline()) {
            offlineFixedLoss += k3 / 4.0f;
        }
    }
    // 未限功率前的总估算功率，从离线轮的固定损耗起算
    sumCmdPower = offlineFixedLoss;

    // 可分配功率：先扣除离线轮固定损耗，负功率轮会“返还预算”
    float allocatablePower = std::max(maxPower - offlineFixedLoss, 0.0f);
    // 正功率轮的总需求（用于按权重切分）
    float sumPowerRequired = 0.0f;
#if USE_DEBUG
    static float newCmdPower;
#endif

    for (int i = 0; i < 4; i++) {
        if (motors[i].offline()) {
            // 离线轮不参与功率需求、误差权重和预算分配
            cmdPower[i] = 0.0f;
            error[i] = 0.0f;
            continue;
        }
        PowerObj *p = objs[i];
        // 单轮功率模型:
        // P = τω + k1|ω| + k2τ² + k3/4
        // 第一项: 有效机械功（驱动/制动）
        // 第二项: 与速度相关损耗（粘性/风阻/机械摩擦等）
        // 第三项: 与电流平方相关损耗（铜损）
        // 第四项: 常量损耗均分到四轮
        cmdPower[i] = p->pidOutput * k0 * p->curAv + fabs(p->curAv) * k1 +
                      p->pidOutput * k0 * p->pidOutput * k0 * k2 + k3 / static_cast<float>(4);
        sumCmdPower += cmdPower[i];
        error[i] = fabs(p->setAv - p->curAv);

        // 对于负功率轮（回收/制动），其“功率需求”可视作给其他轮释放预算
        if (floatEqual(cmdPower[i], 0.0f) || cmdPower[i] < 0.0f) {
            allocatablePower += -cmdPower[i];
        } else {
            sumError += error[i];
            sumPowerRequired += cmdPower[i];
        }
    }

    // LOG_INFO(
    //     "sum power: %f, Max power: %f, Measured: %f, CapEnergy: %d, buffer_energy %d %d\n",
    //     sumCmdPower,
    //     maxPower,
    //     measuredPower,
    //     robot_set->super_cap_info.capEnergy,
    //     robot_set->referee_info.game_robot_status_data.robot_id,
    //     robot_set->referee_info.game_robot_status_data.robot_level);

    // LOG_INFO("k1 %f k2 %f k3 %f max %f\n", k1, k2, k3, maxPower);

    // LOG_INFO("referee level %d\n",
    // robot_set->referee_info.game_robot_status_data.robot_level);

    //      update power status
    powerStatus.maxPowerLimited = maxPower;
    powerStatus.sumPowerCmd_before_clamp = sumCmdPower;

    // 仅在总需求超过上限时触发“限功率重分配”
    if (sumCmdPower > maxPower) {
        float errorConfidence;
        // 权重混合策略:
        // - 误差大时，优先保控制误差（error 权重大）
        // - 误差小时，按功率占比分配（prop 权重大）
        if (sumError > error_powerDistribution_set) {
            errorConfidence = 1.0f;
        } else if (sumError > prop_powerDistribution_set) {
            errorConfidence = std::clamp(
                (sumError - prop_powerDistribution_set) /
                    (error_powerDistribution_set - prop_powerDistribution_set),
                0.0f,
                1.0f);
        } else {
            errorConfidence = 0.0f;
        }
        for (int i = 0; i < 4; i++) {
            if (motors[i].offline()) {
                newTorqueCurrent[i] = 0.0f;
                continue;
            }
            PowerObj *p = objs[i];

            // 负功率轮不参与削峰，保持原输出（它本身就在降总功率）
            if (floatEqual(cmdPower[i], 0.0f) || cmdPower[i] < 0.0f) {
                newTorqueCurrent[i] = p->pidOutput;
                continue;
            }

            // 综合权重 = 误差权重 与 功率权重 的线性插值
            // sumError/sumPowerRequired 可能为 0（例如轮子堵转但目标速度等于当前速度），
            // 直接相除会产生 NaN，进而让电流指令变成 NaN
            float powerWeight_Error =
                sumError > 1e-5f ? fabs(p->setAv - p->curAv) / sumError : 0.0f;
            float powerWeight_Prop =
                sumPowerRequired > 1e-5f ? cmdPower[i] / sumPowerRequired : 0.0f;
            float powerWeight = errorConfidence * powerWeight_Error +
                                (1.0f - errorConfidence) * powerWeight_Prop;

            // 将“目标功率 = powerWeight * allocatablePower”代回功率二次式求电流:
            // k2*(k0*u)^2 + ω*(k0*u) + (k1|ω| + k3/4 - P_target) = 0
            // 判别式写成 delta，后续按根求新的电流指令
            float delta =
                p->curAv * p->curAv - 4.0f * k2 *
                                          (k1 * fabs(p->curAv) + k3 / static_cast<float>(4) -
                                           powerWeight * allocatablePower);

            // delta=0: 重根
            if (floatEqual(delta, 0.0f))
            {
                newTorqueCurrent[i] = -p->curAv / (2.0f * k2) / k0;
            } else if (delta > 0.0f)
            {
                // 有两个实根时，按原 pidOutput 的符号选同向根，避免控制方向翻转
                newTorqueCurrent[i] = p->pidOutput > 0.0f
                                          ? (-p->curAv + sqrtf(delta)) / (2.0f * k2) / k0
                                          : (-p->curAv - sqrtf(delta)) / (2.0f * k2) / k0;
            } else
            {
                // 无实根时退化到抛物线顶点，保证有界
                newTorqueCurrent[i] = -p->curAv / (2.0f * k2) / k0;
            }

            // 最后再做一次电流硬限幅
            newTorqueCurrent[i] =
                std::clamp(newTorqueCurrent[i], -p->pidMaxOutput, p->pidMaxOutput);
        }
    } else {
        for (int i = 0; i < 4; i++) {
            newTorqueCurrent[i] = motors[i].offline() ? 0.0f : objs[i]->pidOutput;
        }
    }

    // 调试用途：统计限幅后的总估算功率
    float newCmdPower = 0.0f;
    for (int i = 0; i < 4; i++) {
        if (motors[i].offline()) {
            // 离线轮只保留固定损耗，不估算电流项
            newCmdPower += k3 / 4.0f;
            continue;
        }
        PowerObj *p = objs[i];
        newCmdPower += newTorqueCurrent[i] * k0 * p->curAv + fabs(p->curAv) * k1 +
                       newTorqueCurrent[i] * k0 * newTorqueCurrent[i] * k0 * k2 + k3 / 4.0f;
    }
    // 该函数每 2 ms 调用一次，日志限频，避免热路径阻塞
    static auto lastLogTime = std::chrono::steady_clock::now();
    const auto logNow = std::chrono::steady_clock::now();
    if (logNow - lastLogTime >= std::chrono::milliseconds(500)) {
        lastLogTime = logNow;
        LOG_INFO(
            "sumPower: %f, NewCMDPower power: %f, measuredPower: %f, capEnergy: %d\n",
            sumPowerRequired,
            newCmdPower,
            robot_set->super_cap_info.chassisPower,
            robot_set->super_cap_info.capEnergy);
    }

    //      #endif

    return newTorqueCurrent; 
}

   [[noreturn]] void Manager::powerDaemon () {
        // RLS 的输入向量:
        // samples[0] = Σ|ω|, samples[1] = Στ²
        static Math::Matrixf<2, 1> samples;
        static Math::Matrixf<2, 1> params;
        // effectivePower = Σ(τω) 项，代表有用机械功
        static float effectivePower = 0;
        //std::ofstream outputFile("log.txt");

        isInitialized = true;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        lastUpdateTick = clock();

        while (true) {
            // 使用powerUpperLimit
            setMode(1);
            float torqueConst = 0.3f; // 输出轴扭矩常数 (N·m/A)
            float k0 = torqueConst * 20.0f / 16384.0f; // 输出轴扭矩 (N·m/指令单位)
            size_t now = clock();

            // 电容能量估计:
            // 协议里 capEnergy 为 0~255，内部映射到 0~2100 的工程量
            estimatedCapEnergy = robot_set->super_cap_info.capEnergy / 255.0f * 2100.0f;

            // 注意: 这里对 capEnergy 取 sqrt，因为发来之前平方过
            // 对应地，目标 fullBuffSet/baseBuffSet 也在相同域上比较，便于 PD 调节。
            powerBuff = sqrtf(robot_set->super_cap_info.capEnergy);

            fullBuffSet = capFullBuffSet;  // 250
            baseBuffSet = capBaseBuffSet;  // 100

            // 基础功率上限：取电容反馈的 chassisPowerlimit，并加保底下界
            refereeMaxPower = fmax(
                robot_set->super_cap_info.chassisPowerlimit,
                CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD);
            // 上限=裁判上限+电容额外输出
            powerUpperLimit = refereeMaxPower + MAX_CAP_POWER_OUT;
            // LOG_INFO("powerUpperLimit:%f, refereeMaxpower: %f\n", powerUpperLimit, refereeMaxPower);
            powerLowerLimit = 50;

            // 能量环:
            // 目标与当前都在 sqrt 域，PD 输出作为“需要扣减的功率”
            powerPD_base.set(sqrtf(baseBuffSet));
            powerPD_full.set(sqrtf(fullBuffSet));
            baseMaxPower = fmax(refereeMaxPower - powerPD_base.out, MIN_MAXPOWER_CONFIGURED);
            fullMaxPower = fmax(refereeMaxPower - powerPD_full.out, MIN_MAXPOWER_CONFIGURED);

            // 基于反馈电流与转速，计算模型输入
            effectivePower = 0;
            samples[0][0] = 0;
            samples[1][0] = 0;
            bool any_offline = false;
            for (int i = 0; i < 4; i++) {
                //LOG_INFO("%d", motors[i].motor_measure_.given_current);

                // 离线电机的反馈已过期，不参与模型/RLS 统计
                if (motors[i].offline()) {
                    any_offline = true;
                    continue;
                }

                // 统一用输出轴角速度，与 getControlledOutput 的 curAv 同侧
                const float output_av = motors[i].data_.output_angular_velocity;

                // τω 项（输出轴扭矩 × 输出轴角速度）
                effectivePower += motors[i].motor_measure_.given_current * k0 * output_av;
                // |ω| 累加项（对应 k1）
                samples[0][0] += fabsf(output_av);
                // τ² 累加项（对应 k2）
                samples[1][0] += motors[i].motor_measure_.given_current * k0 *
                                 motors[i].motor_measure_.given_current * k0;
            }
            // 总估计功率 = k1*Σ|ω| + k2*Στ² + Στω + k3
            estimatedPower = k1 * samples[0][0] + k2 * samples[1][0] + effectivePower + k3;

            // 电容实测底盘功率
            measuredPower = robot_set->super_cap_info.chassisPower;
            // NOTE: log k1 k2 k3
            // LOG_INFO(
            //     "%f %f %f %f %f %f\n", measuredPower, effectivePower, estimatedPower, k1, k2,
            //     k3);

            // NOTE: log PIDs
            // LOG_INFO(
            //    "%f, %f, %f, %f, %f %d\n",
            //    sqrtf(baseBuffSet),
            //    powerBuff,
            //    refereeMaxPower,
            //    powerPD_base.out,
            //    baseMaxPower,
            //    robot_set->super_cap_info.capEnergy);

            // NOTE: log super_cat_info
            // LOG_INFO(
            //    "%d %f %d\n",
            //    robot_set->super_cap_info.capEnergy,
            //    robot_set->super_cap_info.chassisPower,
            //    robot_set->super_cap_info.chassisPowerlimit);

            // NOTE: for dumping log and draw purpose
            // printf("%f, %f\n", baseMaxPower, fullMaxPower);
            // outputFile << refereeMaxPower << ", " << baseMaxPower << "\n" << std::flush;
            //outputFile << baseMaxPower << ", " << fullMaxPower << "\n" << std::flush;

            // 刷新对外状态（供 UI/调试）
            powerStatus.userConfiguredMaxPower = userConfiguredMaxPower;
            powerStatus.effectivePower = effectivePower;
            powerStatus.powerLoss = measuredPower - effectivePower;
            powerStatus.efficiency =
                fabsf(measuredPower) > 1e-3f
                    ? std::clamp(effectivePower / measuredPower, 0.0f, 1.0f)
                    : 0.0f;
            powerStatus.estimatedCapEnergy =
                static_cast<uint8_t>(estimatedCapEnergy / 2100.0f * 255.0f);

            // 刷新错误标志（供状态显示/调试）
            error = 0;
            for (int i = 0; i < 4; ++i) {
                if (motors[i].offline()) {
                    setErrorFlag(error, Manager::MotorDisconnect);
                    break;
                }
            }
            powerStatus.error = static_cast<Manager::ErrorFlags>(error);

            // RLS 在线辨识:
            // measuredPower - effectivePower - k3 = k1*Σ|ω| + k2*Στ²
            // 小功率区间不更新，降低噪声放大
            if (!any_offline && fabs(measuredPower) > 5.0f) {
                params = rls.update(samples, measuredPower - effectivePower - k3);
                k1 = fmax(params[0][0],
                          1e-5f);  // In case the k1 diverge to negative number
                k2 = fmax(params[1][0],
                          1e-5f);  // In case the k2 diverge to negative number
            }

            lastUpdateTick = now;

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    /**
     * @implements
     */
    void Manager::init(const std::shared_ptr<Robot::Robot_set> &robot) {
        if (isInitialized)
            return;

        robot_set = robot;
        // 初始化阶段先给一组保守默认值，避免上电初期无界
        powerUpperLimit = CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD;
        // 能量环 PID 的 ref 绑定到 powerBuff（当前能量）
        powerPD_base = Pid::PidPosition(powerPD_base_pid_config, powerBuff);
        powerPD_full = Pid::PidPosition(powerPD_full_pid_config, powerBuff);
    }
}  // namespace Power
