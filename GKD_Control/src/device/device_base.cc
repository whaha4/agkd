#include "device/deviece_base.hpp"
#include "robot_type_config.hpp"

using namespace std::chrono;

namespace Device
{
    DeviceBase::DeviceBase() : last_time(steady_clock::now() - 10s), offline_time(Config::DEFAULT_OFFLINE_TIME) {
    }

    DeviceBase::DeviceBase(uint32_t offline_time_t)
        : last_time(steady_clock::now() - 10s),
          offline_time(offline_time_t) {
    }

    bool DeviceBase::offline() const {
        return duration_cast<milliseconds>(steady_clock::now() - last_time).count() >= offline_time;
    }

    void DeviceBase::update_time() {
        last_time = steady_clock::now();
    }

    void DeviceBase::set_offline_time(uint32_t offline_time_ms) {
        if (offline_time_ms > 0) {
            offline_time = offline_time_ms;
        }
    }
}  // namespace Device
