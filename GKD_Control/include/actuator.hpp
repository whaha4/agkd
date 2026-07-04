#pragma once

#include "controller.hpp"

class Actuator
{
   public:
    ControllerList controller;
    Actuator(const Actuator &other) = delete;

    Actuator() = default;

    bool operator=(const Actuator &other) = delete;
    virtual void set(float x) = 0;
    virtual ~Actuator() = default;

    template<ControllerBase Ctrl>
    void setCtrl(Ctrl &&ctrl) {
        controller = std::forward<Ctrl>(ctrl);
    }

    template<ControllerBase Ctrl>
    void pushFrontCtrl(Ctrl &&ctrl) {
        controller = std::forward<Ctrl>(ctrl) >> std::move(controller);
    }

    template<ControllerBase Ctrl>
    void pushBackCtrl(Ctrl &&ctrl) {
        controller = std::move(controller) >> std::forward<Ctrl>(ctrl);
    }
};

template<typename T>
concept ActuatorBase = std::is_base_of_v<Actuator, T>;

// template<ControllerBase Ctrl, ActuatorBase Act>
// Act operator>>(Ctrl &&ctrl, Act &&act) {
//     Act res = act;
//     res.controller = std::forward<Ctrl>(ctrl) >> std::forward<Act>(act.controller);
//     return res;
// }

template<ActuatorBase Act>
void operator>>(float x, Act &act) {
    act.set(x);
}
