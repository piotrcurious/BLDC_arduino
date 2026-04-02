#include "Arduino.h"
#include "../BLDC_speed_control_extra_regen.ino"

void run_scenario(const char* name, int iterations, int spd_target, float load, float friction) {
    std::cout << "\n>>> Scenario: " << name << " (Spd:" << spd_target << " Load:" << load << " Fric:" << friction << ")" << std::endl;
    analogWrite(A1, spd_target);
    setMockLoad(load);
    setMockFriction(friction);
    for (int i = 0; i < iterations; i++) {
        loop();
        sync_sim();
        if (i % (iterations/5) == 0) {
           std::cout << "  - Progress " << (i*100/iterations) << "%" << std::endl;
        }
    }
}

int main() {
    setup();

    // 1. Spin up
    run_scenario("Spin Up", 200, 800, 0.0, 0.00001);

    // 2. Spin up with increasing friction
    run_scenario("Increasing Friction", 200, 800, 0.0, 0.001);

    // 3. Stall
    run_scenario("Stall (High Load)", 200, 800, 0.5, 0.00001);

    // 4. Overpowering (Negative Load / Reverse Torque)
    run_scenario("Overpowered (Opposite Direction)", 200, 800, -0.2, 0.00001);

    // 5. Regen Braking
    run_scenario("Regen Braking", 200, 100, 0.0, 0.00001);

    // 6. Spin down
    run_scenario("Spin Down", 200, 512, 0.0, 0.00001);

    std::cout << "\nAll scenarios completed." << std::endl;
    return 0;
}
