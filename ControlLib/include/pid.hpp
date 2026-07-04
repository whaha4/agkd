#pragma once
#include <controller.hpp>

namespace Pid
{
    /**
     * @brief          pid config data
     * @param[in]      PID: 0: kp, 1: ki, 2:kd
     * @param[in]      max_out: pid最大输出
     * @param[in]      max_iout: pid最大积分输出
     * @retval         none
     */

    struct PidConfig
    {

        // PID 三参数
        float kp;
        float ki;
        float kd;

        float max_out;   // 最大输出
        float max_iout;  // 最大积分输出
    };

    class PidPosition final : public PidConfig, public Controller
    {
    public:
        const float &ref;
        ~PidPosition() override = default;
        explicit PidPosition(const PidConfig &config, const float &ref) : PidConfig(config), ref(ref) {}
        void set(float set_v) override;
        void clean();

    public:
        float Pout = 0.f;
        float Iout = 0.f;
        float Dout = 0.f;
        float Dbuf = 0.f;   // 微分项 0最新 1上一次 2上上次
        float error[2] = {};  // 误差项 0最新 1上一次 2上上次
    };



    class PidRad final : public PidConfig, public Controller
    {
    public:
        const float &ref;
        explicit PidRad(const PidConfig &config, const float &ref) : PidConfig(config), ref(ref) {};
        ~PidRad() override = default;
        void set(float set) override;

    public:
        float Pout = 0.f;
        float Iout = 0.f;
        float Dout = 0.f;
        float err = 0.f;
        float last_err = 0.f;
    };
}  // namespace Pid