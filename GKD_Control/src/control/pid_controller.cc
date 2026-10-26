#include "pid_controller.hpp"

#include <cmath>

#include "user_lib.hpp"

namespace Pid
{
    void PidPosition::set(const fp32 set_v) {
        error[1] = error[0];
        error[0] = (set_v - ref);

        Pout = kp * error[0];
        Dbuf = (error[0] - error[1]);
        Dout = kd * Dbuf;

        const fp32 pd = Pout + Dout;
        const fp32 unsat = pd + Iout;

        // 条件积分: 输出已饱和且误差仍在往饱和方向推时，本拍不再积分
        if (!(std::abs(unsat) > max_out && error[0] * unsat > 0.f)) {
            Iout += ki * error[0];
        }
        Iout = std::clamp(Iout, -max_iout, max_iout);

        out = std::clamp(pd + Iout, -max_out, max_out);
    }

    void PidPosition::clean() {
        error[0] = error[1] = 0;
        Dbuf = 0.f;
        Pout = Iout = Dout = 0;
        out = 0;
    }

    void PidRad::set(const fp32 set) {
        last_err = err;

        err = UserLib::rad_format(set - ref);
        Pout = kp * err;
        Dout = kd * (err - last_err);

        const fp32 pd = Pout + Dout;
        const fp32 unsat = pd + Iout;

        // 条件积分: 输出已饱和且误差仍在往饱和方向推时，本拍不再积分
        if (!(std::abs(unsat) > max_out && err * unsat > 0.f)) {
            Iout += ki * err;
        }
        Iout = std::clamp(Iout, -max_iout, max_iout);

        out = std::clamp(pd + Iout, -max_out, max_out);
    }

    void Invert::set(const fp32 set) {
        out = set * dir;
    }
}  // namespace Pid
