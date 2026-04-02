#include "Arduino.h"
#include <iostream>

// Test MPPT performance
#include "../mppt_BLDC.ino"

int main() {
    setup();
    std::cout << "TEST: MPPT Performance START" << std::endl;

    // Simulate varying bus voltage (e.g. solar panel)
    for (int v = 120; v >= 60; v -= 10) {
        float bus_v = v / 10.0;
        std::cout << "  - Bus Voltage: " << bus_v << "V" << std::endl;
        // In this mock, we'll manually send the BUS CMD via sync_sim extension or just use CMD
        // For now, let's just loop and observe tracking
        for (int i = 0; i < 100; i++) {
            loop();
            sync_sim();
        }
        std::cout << "    Power: " << motor_power << "W, Step: " << commutation_step << std::endl;
    }

    std::cout << "TEST: MPPT Performance END" << std::endl;
    return 0;
}
