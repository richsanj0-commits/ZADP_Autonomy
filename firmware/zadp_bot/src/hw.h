// hw.h : motor driver (TB6612FNG via LEDC PWM) and quadrature encoders (PCNT or GPIO interrupts).
#pragma once

#include <stdint.h>

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
const char* encoderBackend();

// Optional yaw-rate gyro (USE_GYRO). gyroBegin() in setup() with the robot still; gyroRate() only from the
// control task. Both return false when there is no gyro: heading then comes from the encoders.
bool gyroBegin();
bool gyroOk();
bool gyroRate(float& radPerS);
void gyroCheckSign(float encRate);     // control task, every tick: yaw rate from the encoders, rad/s          // "pcnt" or "gpio-isr" (serial banner)

}  // namespace hw
