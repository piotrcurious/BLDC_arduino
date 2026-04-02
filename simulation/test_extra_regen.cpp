#include "Arduino.h"
#include "../BLDC_speed_control_extra_regen.ino"

int main() {
    setup();
    for (int i = 0; i < 1000; i++) {
        loop();
        sync_sim();
    }
    return 0;
}
