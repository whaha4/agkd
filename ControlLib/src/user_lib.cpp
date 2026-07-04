#include "user_lib.hpp"

#include <thread>
#include <chrono>
#include <cmath>

namespace UserLib
{
    void sleep_ms(const uint32_t dur) {
        std::this_thread::sleep_for(std::chrono::milliseconds(dur));
    }

    float rad_format(const float ang) {
        const float ans = fmodf(ang + M_PIf, M_PIf * 2.f);
        return (ans < 0.f) ? ans + M_PIf : ans - M_PIf;
    }
}  // namespace UserLib
