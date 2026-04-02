#include "Arduino.h"
#include "../mppt_BLDC.ino"

int main() {
    setup();
    for (int i = 0; i < 100; i++) {
        loop();
        sync_sim();
    }
    return 0;
}
