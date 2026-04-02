#include "Arduino.h"
#include "../BLDC_speed_control.ino"

int main() {
    setup();
    for (int i = 0; i < 100; i++) {
        loop();
        sync_sim();
    }
    return 0;
}
