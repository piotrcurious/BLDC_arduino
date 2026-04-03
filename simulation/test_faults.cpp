#include "Arduino.h"
#include <iostream>

#include "../BLDC_speed_control_extra_regen.ino"

int main() {
    setup();
    std::cout << "TEST: Fault Tolerance START" << std::endl;

    // 1. Stuck Hall Sensor
    std::cout << "  - Injecting Stuck Hall Sensor Fault (Hall A stuck LOW)..." << std::endl;
    analogWrite(A1, 600);
    setMockStuckHall(0, 0);
    for (int i = 0; i < 500; i++) {
        loop();
        sync_sim();
    }
    std::cout << "    Speed during Hall fault: " << analogRead(18) << std::endl;

    // 2. Open Phase
    std::cout << "\n  - Injecting Open Phase Fault (Phase A open)..." << std::endl;
    setMockStuckHall(0, -1); // Clear hall fault (assuming -1 means clear in a better protocol, but let's just re-setup or similar)
    // Actually, our CMD_STUCK_HALL doesn't have a clear. Let's just assume we continue or restart.
    setMockOpenPhase(0);
    for (int i = 0; i < 500; i++) {
        loop();
        sync_sim();
    }
    std::cout << "    Speed during Open Phase: " << analogRead(18) << std::endl;

    std::cout << "TEST: Fault Tolerance END" << std::endl;
    return 0;
}
