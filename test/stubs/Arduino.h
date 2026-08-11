// ============================================================================
// FILE: test/stubs/Arduino.h   (NATIVE HOST BUILD ONLY — never compiled for ESP32)
// Minimal Arduino/ESP32 shim so the control logic in src/ can be compiled and
// driven on a PC. Only what src/ShiftScheduler|SolenoidDriver|EngineProfile|
// AdaptiveMemory|DtcManager actually touch is provided.
//
// The two things that make host testing possible:
//   1. TIME IS A VARIABLE (g_now_ms). Tests advance it explicitly, so a 600 ms
//      backstop or a 1500 ms engage grace runs in microseconds and is exactly
//      reproducible — no sleeping, no flakiness.
//   2. PWM/GPIO WRITES ARE RECORDED (g_pwm/g_dio) instead of touching hardware.
//      That recording is the assertion surface: which routing solenoid was
//      kicked, what SPC/MPC was commanded, in what order.
// ============================================================================
#pragma once

#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <cstdarg>
#include <math.h>
#include <string>

// ---------------------------------------------------------------------------
// Controllable time. millis() never advances on its own.
// ---------------------------------------------------------------------------
inline uint32_t g_now_ms = 0;
inline unsigned long millis() { return (unsigned long)g_now_ms; }
inline unsigned long micros() { return (unsigned long)g_now_ms * 1000UL; }
inline void delay(unsigned long ms) { g_now_ms += (uint32_t)ms; }

// ---------------------------------------------------------------------------
// FreeRTOS surface used by the scheduler/driver (1 tick == 1 ms, as configured).
// ---------------------------------------------------------------------------
typedef uint32_t TickType_t;
#define portTICK_PERIOD_MS 1
inline TickType_t xTaskGetTickCount() { return (TickType_t)g_now_ms; }

typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
inline void portENTER_CRITICAL(portMUX_TYPE*) {}
inline void portEXIT_CRITICAL(portMUX_TYPE*) {}

// ---------------------------------------------------------------------------
// GPIO / LEDC capture. g_pwm[pin] holds the last duty written (0-255).
// ---------------------------------------------------------------------------
#define OUTPUT          1
#define INPUT           0
#define INPUT_PULLDOWN  2
#define INPUT_PULLUP    3
#define HIGH            1
#define LOW             0

#define HW_MAX_PIN 64
inline uint16_t g_pwm[HW_MAX_PIN];   // last ledcWrite duty per pin
inline uint8_t  g_dio[HW_MAX_PIN];   // last digitalWrite level per pin

inline void hwResetPins() {
    for (int i = 0; i < HW_MAX_PIN; i++) { g_pwm[i] = 0; g_dio[i] = 0; }
}

inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t pin, uint8_t v) { if (pin < HW_MAX_PIN) g_dio[pin] = v; }
inline int  digitalRead(uint8_t) { return 0; }
inline int  analogRead(uint8_t) { return 0; }
inline bool ledcAttach(uint8_t, uint32_t, uint8_t) { return true; }
inline bool ledcWrite(uint8_t pin, uint32_t duty) {
    if (pin < HW_MAX_PIN) g_pwm[pin] = (uint16_t)duty;
    return true;
}

// ---------------------------------------------------------------------------
// Arduino helpers. `constrain` stays a MACRO (as on ESP32) so mixed-type call
// sites behave identically; `map` stays a function so it cannot collide with
// std::map inside the Preferences stub.
// ---------------------------------------------------------------------------
#define constrain(a, l, h) ((a) < (l) ? (l) : ((a) > (h) ? (h) : (a)))
inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
    if (in_max == in_min) return out_min;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ---------------------------------------------------------------------------
// Serial — swallowed. Set g_serial_echo=true when debugging a test locally.
// ---------------------------------------------------------------------------
inline bool g_serial_echo = false;
struct SerialStub {
    void begin(unsigned long) {}
    void print(const char* s)   { if (g_serial_echo) std::printf("%s", s); }
    void print(int v)           { if (g_serial_echo) std::printf("%d", v); }
    void print(unsigned v)      { if (g_serial_echo) std::printf("%u", v); }
    void print(long v)          { if (g_serial_echo) std::printf("%ld", v); }
    void print(unsigned long v) { if (g_serial_echo) std::printf("%lu", v); }
    void print(float v)         { if (g_serial_echo) std::printf("%f", (double)v); }
    void print(double v)        { if (g_serial_echo) std::printf("%f", v); }
    void println()              { if (g_serial_echo) std::printf("\n"); }
    template <class T> void println(T v) { print(v); println(); }
    int printf(const char* fmt, ...) {
        if (!g_serial_echo) return 0;
        va_list ap; va_start(ap, fmt);
        int n = std::vprintf(fmt, ap);
        va_end(ap); return n;
    }
};
inline SerialStub Serial;

// ---------------------------------------------------------------------------
// Arduino String — only the concat + c_str() usage in AdaptiveMemory.
// ---------------------------------------------------------------------------
class String {
  public:
    std::string s;
    String() {}
    String(const char* p) : s(p ? p : "") {}
    String(int v) : s(std::to_string(v)) {}
    String(unsigned int v) : s(std::to_string(v)) {}
    String(long v) : s(std::to_string(v)) {}
    const char* c_str() const { return s.c_str(); }
    String& operator+=(const String& o) { s += o.s; return *this; }
};
inline String operator+(const String& a, const String& b) {
    String r; r.s = a.s + b.s; return r;
}
