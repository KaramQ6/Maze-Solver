#ifndef TEST_ARDUINO_H
#define TEST_ARDUINO_H

#include <stdint.h>
#include <stdlib.h>
#include <algorithm>

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2

extern unsigned long test_time_ms;
extern int test_pins[32];
extern int test_pwm[32];
extern int test_channels[32];
void test_advance_time(unsigned long duration_ms);

inline unsigned long millis() { return test_time_ms; }
inline unsigned long micros() { return test_time_ms * 1000; }
inline void delay(unsigned long duration_ms) { test_advance_time(duration_ms); }
inline void pinMode(int, int) {}
inline void digitalWrite(int pin, int value) { test_pins[pin] = value; }
inline double ledcSetup(int, double frequency, int) { return frequency; }
inline void ledcAttachPin(int pin, int channel) { test_channels[channel] = pin; }
inline bool ledcAttach(int pin, int, int) { test_channels[pin] = pin; return true; }
inline bool ledcWrite(int channel, uint32_t duty) { test_pwm[channel] = (int)duty; return true; }
template <typename T> inline T constrain(T value, T low, T high) {
    return std::max(low, std::min(value, high));
}
using std::min;

#endif
