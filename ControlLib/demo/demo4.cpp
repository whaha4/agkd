#include <iostream>

#include "controller.hpp"
#include "actuator.hpp"

class Mul : public Controller {
public:
    float k = 0;

    Mul(float k_) : k(k_) {}
    void set(float x) override {
        out = x * k;
    }
    ~Mul() override = default;
};

class Output : public Actuator {
public:
    void set_directly(float x) override {
        std::cout << "get: " << x << std::endl;
    }
    ~Output() override = default;
};

int main() {
    Mul mul(2.21);
    Output output;
    2 >> mul >> output;
}