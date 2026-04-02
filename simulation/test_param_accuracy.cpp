#include "Arduino.h"
#include <iostream>

#include "../BLDC_max_current.ino"

int main() {
    // Test for 1mH
    std::cout << "TEST: Parameter Accuracy (L=1mH) START" << std::endl;
    setMockParamL(0.001);
    setMockParamR(0.1);
    setup();
    // In setup, max_current measures inductance
    std::cout << "TEST: Parameter Accuracy (L=1mH) END" << std::endl;

    // Test for 5mH
    std::cout << "\nTEST: Parameter Accuracy (L=5mH) START" << std::endl;
    L_MOTOR = 0.001; // reset internal global if needed, though setup should do it
    setMockParamL(0.005);
    setup();
    std::cout << "TEST: Parameter Accuracy (L=5mH) END" << std::endl;

    return 0;
}
