#ifndef ARDUINO_H
#define ARDUINO_H

#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <cstdint>

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352

#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19

#define CHANGE 1
#define FALLING 2
#define RISING 3

#define B000 0
#define B001 1
#define B010 2
#define B011 3
#define B100 4
#define B101 5
#define B110 6
#define B111 7

typedef unsigned char byte;
typedef bool boolean;

// Forward declarations
int readHallSensors();
int determineMode();
float measureCurrent();
void encoderISR();
void initPWM();
void setPhasePWM();
void measureInductance();
void findMaxPowerPoint();

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
void analogWrite(uint8_t pin, int val);

void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
unsigned long millis();
unsigned long micros();

void attachInterrupt(uint8_t interruptNum, void (*userFunc)(void), int mode);
void detachInterrupt(uint8_t interruptNum);
uint8_t digitalPinToInterrupt(uint8_t pin);

void analogWriteFrequency(uint8_t pin, float frequency);
void analogWriteResolution(int res);

long map(long x, long in_min, long in_max, long out_min, long out_max);
int max(int a, int b);
int min(int a, int b);

class Serial_ {
public:
    void begin(unsigned long baud);
    void print(const char* s);
    void print(std::string s) { std::cout << s; }
    void print(int n, int base = 10);
    void print(float f, int prec = 2);
    void print(double d, int prec = 2) { print((float)d, prec); }
    void println(const char* s);
    void println(std::string s) { std::cout << s << std::endl; }
    void println(int n, int base = 10);
    void println(float f, int prec = 2);
    void println(double d, int prec = 2) { println((float)d, prec); }
    void println();
};

extern Serial_ Serial;

void sync_sim();
void setMockLoad(float torque);
void setMockFriction(float b);

#endif
