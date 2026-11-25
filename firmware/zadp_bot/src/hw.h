// hw.h : motor driver (TB6612FNG via LEDC PWM), quadrature encoders (PCNT or GPIO interrupts), MPU6050 gyro.
#pragma once

#include <stdint.h>

#include "config.h"

namespace hw {

// Motors. motorsBegin() sets every driver pin and writes zero (short brake) BEFORE anything else runs.
// motorOut() is called only by the control task after setup().
void motorsBegin();
void motorOut(bool left, int pwm);     // signed PWM, + = wheel forward, clamped to +-PWM_MAX
void motorsOff();

// Encoders: cumulative 32-bit counts, forward positive (ENC_x_DIR applied).
// encodersRead() is called only by the control task after setup() (the PCNT unwrap state is not shared).
void encodersBegin();
void encodersRead(int32_t& left, int32_t& right);
const char* encoderBackend();          // "pcnt" or "gpio-isr" (serial banner)

// Yaw-rate gyro, detected at boot. gyroBegin() in setup() with the robot still; gyroRate() and gyroCheckSign()
// only from the control task. false = no gyro (or lost): heading then comes from the encoders.
bool gyroBegin();
bool gyroOk();
uint8_t gyroWhoAmI();                  // WHO_AM_I register (0x75) read at boot, 0 = no answer
const char* gyroStatus();              // "on (...)" or "off (reason)"
bool gyroRate(float& radPerS);
void gyroCheckSign(float encRate);     // yaw rate from the encoders, rad/s

#if BENCH_SIM
int benchWheelPwm(bool left);          // bench build only: last signed PWM given to motorOut() (motors.cpp)
#endif

}  // namespace hw
