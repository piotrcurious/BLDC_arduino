#include "Arduino.h"
#include <iostream>
#include <cmath>

#include "../BLDC_speed_control_extra_regen.ino"

int main() {
    setup();
    std::cout << "TEST: Robustness and Noise START" << std::endl;

    // 1. Noise Injection
    std::cout << "  - Testing with 5% Hall sensor noise..." << std::endl;
    analogWrite(A1, 600);
    setMockNoise(0.05);
    for (int i = 0; i < 500; i++) {
        loop();
        sync_sim();
    }
    std::cout << "    Speed at 5% noise: " << analogRead(18) << std::endl;

    // 2. Oscillating Load
    std::cout << "  - Testing with oscillating load..." << std::endl;
    setMockNoise(0.0);
    for (int i = 0; i < 500; i++) {
        float load = 0.1 * sin(i * 0.1);
        setMockLoad(load);
        loop();
        sync_sim();
    }
    std::cout << "    Final Speed after oscillating load: " << analogRead(18) << std::endl;

    std::cout << "TEST: Robustness and Noise END" << std::endl;
    return 0;
}
