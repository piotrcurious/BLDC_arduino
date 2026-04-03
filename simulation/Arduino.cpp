#include "Arduino.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <iomanip>
#include <map>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstdio>

struct PinState {
    int mode;
    int digital_val;
    int analog_val;
    float frequency;
};

static std::map<int, PinState> pins;
static std::map<int, void (*)()> interrupts;

static int pipe_in = -1;
static int pipe_out = -1;

void sync_sim() {
    if (pipe_in == -1) {
        std::cout << "Arduino: Opening pipes..." << std::endl;
        pipe_out = open("/tmp/arduino_in", O_WRONLY);
        pipe_in = open("/tmp/arduino_out", O_RDONLY);
        if (pipe_in == -1 || pipe_out == -1) {
            return;
        }
        std::cout << "Arduino: Pipes opened." << std::endl;
    }

    // Write all outputs to sim
    for (auto const& [pin, state] : pins) {
        if (state.mode == OUTPUT || pin == A1) {
            std::string s = "P" + std::to_string(pin) + "D" + std::to_string(state.digital_val) + "A" + std::to_string(state.analog_val) + "\n";
            write(pipe_out, s.c_str(), s.length());
        }
    }
    write(pipe_out, "SYNC\n", 5);

    // Read inputs from sim
    char buf[1024];
    int n;
    std::string s = "";
    while (true) {
        n = read(pipe_in, buf, sizeof(buf)-1);
        if (n <= 0) break;
        buf[n] = 0;
        s += buf;
        if (s.find("ACK\n") != std::string::npos) break;
    }

    if (s.find("ACK\n") != std::string::npos) {
        size_t pos = 0;
        while ((pos = s.find('I', pos)) != std::string::npos) {
            size_t end = s.find('\n', pos);
            if (end == std::string::npos) break;
            std::string line = s.substr(pos, end - pos);
            int p, d, a;
            if (sscanf(line.c_str(), "I%dD%dA%d", &p, &d, &a) == 3) {
                // Only update pins that are NOT being driven as OUTPUT by the sketch
                // Exception: Always update sensors A0, Hall pins, and telemetry pins
                if (pins[p].mode == INPUT || pins[p].mode == INPUT_PULLUP || p == A0 || (p >= 2 && p <= 4) || p == 8 || p == 9 || (p >= 12 && p <= 13)) {
                    int old_d = pins[p].digital_val;
                    pins[p].digital_val = d;
                    pins[p].analog_val = a;
                    if (old_d != d && interrupts.count(p)) {
                        interrupts[p]();
                    }
                }
            }
            pos = end + 1;
        }
    }
}

static auto sim_start_time = std::chrono::steady_clock::now();

void delay(unsigned long ms) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() < ms) {
        sync_sim();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void delayMicroseconds(unsigned int us) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count() < us) {
        if (us >= 1000) sync_sim();
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
}

unsigned long millis() {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - sim_start_time).count();
}

unsigned long micros() {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(now - sim_start_time).count();
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    if (in_max == in_min) return out_min;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
int max(int a, int b) { return (a > b) ? a : b; }
int min(int a, int b) { return (a < b) ? a : b; }

void Serial_::begin(unsigned long baud) {}
void Serial_::print(const char* s) { std::cout << s; }
void Serial_::print(int n, int base) { std::cout << n; }
void Serial_::print(float f, int prec) { std::cout << std::fixed << std::setprecision(prec) << f; }
void Serial_::println(const char* s) { std::cout << s << std::endl; }
void Serial_::println(int n, int base) { std::cout << n << std::endl; }
void Serial_::println(float f, int prec) { std::cout << std::fixed << std::setprecision(prec) << f << std::endl; }
void Serial_::println() { std::cout << std::endl; }
Serial_ Serial;

void pinMode(uint8_t pin, uint8_t mode) { pins[pin].mode = mode; }
void digitalWrite(uint8_t pin, uint8_t val) { pins[pin].digital_val = val; }
int digitalRead(uint8_t pin) { sync_sim(); return pins[pin].digital_val; }
void analogWrite(uint8_t pin, int val) { pins[pin].analog_val = val; }
int analogRead(uint8_t pin) { sync_sim(); return pins[pin].analog_val; }
void analogWriteFrequency(uint8_t pin, float frequency) { pins[pin].frequency = frequency; }
void analogWriteResolution(int res) {}
void attachInterrupt(uint8_t pin, void (*userFunc)(void), int mode) { interrupts[pin] = userFunc; }
void detachInterrupt(uint8_t pin) { interrupts.erase(pin); }
uint8_t digitalPinToInterrupt(uint8_t pin) { return pin; }

void setMockLoad(float torque) {
    if (pipe_out != -1) {
        std::string s = "CMD_LOAD_" + std::to_string(torque) + "\n";
        write(pipe_out, s.c_str(), s.length());
    }
}
void setMockOpenPhase(int phase) {
    if (pipe_out != -1) {
        std::string s = "CMD_OPEN_PHASE_" + std::to_string(phase) + "\n";
        write(pipe_out, s.c_str(), s.length());
    }
}
void setMockStuckHall(int hall, int val) {
    if (pipe_out != -1) {
        std::string s = "CMD_STUCK_HALL_" + std::to_string(hall) + "_" + std::to_string(val) + "\n";
        write(pipe_out, s.c_str(), s.length());
    }
}
void setMockNoise(float level) {
    if (pipe_out != -1) {
        std::string s = "CMD_NOISE_" + std::to_string(level) + "\n";
        write(pipe_out, s.c_str(), s.length());
    }
}
void setMockParamL(float L) {
    if (pipe_out != -1) {
        std::string s = "CMD_PARAM_L_" + std::to_string(L) + "\n";
        write(pipe_out, s.c_str(), s.length());
    }
}
void setMockParamR(float R) {
    if (pipe_out != -1) {
        std::string s = "CMD_PARAM_R_" + std::to_string(R) + "\n";
        write(pipe_out, s.c_str(), s.length());
    }
}
void setMockFriction(float b) {
    if (pipe_out != -1) {
        std::string s = "CMD_FRIC_" + std::to_string(b) + "\n";
        write(pipe_out, s.c_str(), s.length());
    }
}
