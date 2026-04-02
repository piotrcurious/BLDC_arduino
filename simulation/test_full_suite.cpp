#include "Arduino.h"
#include <iostream>
#include <vector>
#include <string>

// We'll wrap the different sketches to test them in one binary or multiple
// For this suite, we'll focus on the combined scenarios from speed_control_extra_regen
#include "../BLDC_speed_control_extra_regen.ino"

void run_test(const char* name, int iter, int spd, float load, float fric) {
    std::cout << "TEST: " << name << " START" << std::endl;
    analogWrite(A1, spd);
    setMockLoad(load);
    setMockFriction(fric);
    for(int i=0; i<iter; i++){
        loop();
        sync_sim();
    }
    std::cout << "TEST: " << name << " END (I:" << analogRead(A0) << " W:" << analogRead(9) << ")" << std::endl;
}

int main() {
    setup();
    run_test("No-Load Spinup", 200, 800, 0.0, 0.00001);
    run_test("Medium Load", 200, 800, 0.1, 0.00001);
    run_test("Stall Test", 200, 800, 0.5, 0.00001);
    run_test("Regen Test", 200, 200, 0.0, 0.00001);
    return 0;
}
