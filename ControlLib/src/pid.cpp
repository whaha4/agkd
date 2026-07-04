#include "pid.hpp"
#include "user_lib.hpp"

#include <bits/stl_algo.h>

namespace Pid
{
    void PidPosition::set(const float set_v) {
        error[1] = error[0];
        error[0] = (set_v - ref);

        Pout = kp * error[0];
        Iout += ki * error[0];
        Dbuf = (error[0] - error[1]);
        Dout = kd * Dbuf;

        Iout = std::clamp(Iout, -max_iout, max_iout);
        out = Pout + Iout + Dout;
        out = std::clamp(out, -max_out, max_out);
    }

    void PidPosition::clean() {
        error[0] = error[1] = 0;
        Dbuf = 0.f;
        Pout = Iout = Dout = 0;
        out = 0;
    }

    void PidRad::set(const float set) {
        last_err = err;

        err = UserLib::rad_format(set - ref);
        Pout = kp * err;
        Iout += ki * err;
        Dout = kd * (err - last_err);

        Iout = std::clamp(Iout, -max_iout, max_iout);
        out = Pout + Iout + Dout;
        out = std::clamp(out, -max_out, max_out);
    }
}  // namespace Pid