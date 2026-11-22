// motors.cpp : TB6612FNG through LEDC PWM.
// arduino-esp32 3.x addresses PWM by pin (ledcAttach / ledcWrite(pin)); 2.x by channel
// (ledcSetup / ledcAttachPin / ledcWrite(channel)).
#include <Arduino.h>

#include "config.h"
#include "hw.h"

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2   // cores before 2.0 have no version header; they use the 2.x LEDC API
#endif

namespace hw {

static const uint8_t LEDC_CH_L = 0, LEDC_CH_R = 1;
static int lastOut[2] = {0x7fffffff, 0x7fffffff};   // last signed PWM written per wheel (forces the first write)

static void pwmSetup() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(PIN_PWMA, PWM_FREQ_HZ, PWM_BITS);
    ledcAttach(PIN_PWMB, PWM_FREQ_HZ, PWM_BITS);
#else
    ledcSetup(LEDC_CH_L, PWM_FREQ_HZ, PWM_BITS);
    ledcSetup(LEDC_CH_R, PWM_FREQ_HZ, PWM_BITS);
    ledcAttachPin(PIN_PWMA, LEDC_CH_L);
    ledcAttachPin(PIN_PWMB, LEDC_CH_R);
#endif
}

static inline void pwmDuty(uint8_t pin, uint8_t ch, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    (void)ch;
    ledcWrite(pin, duty);
#else
    (void)pin;
    ledcWrite(ch, duty);
#endif
}

// TB6612: IN1 H / IN2 L = one way, L / H = the other, H / H = short brake, L / L = coast.
void motorOut(bool left, int pwm) {
    const int w = left ? 0 : 1;
    if (left ? MOTOR_L_INVERT : MOTOR_R_INVERT) pwm = -pwm;
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    if (pwm < -PWM_MAX) pwm = -PWM_MAX;
    if (pwm == lastOut[w]) return;                               // rule: touch the hardware only on a change
    lastOut[w] = pwm;
    const uint8_t in1 = left ? PIN_AIN1 : PIN_BIN1, in2 = left ? PIN_AIN2 : PIN_BIN2;
    const uint8_t pin = left ? PIN_PWMA : PIN_PWMB, ch = left ? LEDC_CH_L : LEDC_CH_R;
    if (pwm > 0) {
        digitalWrite(in1, HIGH); digitalWrite(in2, LOW); pwmDuty(pin, ch, (uint32_t)pwm);
    } else if (pwm < 0) {
        digitalWrite(in1, LOW); digitalWrite(in2, HIGH); pwmDuty(pin, ch, (uint32_t)(-pwm));
    } else {
        digitalWrite(in1, ZERO_PWM_BRAKE ? HIGH : LOW);
        digitalWrite(in2, ZERO_PWM_BRAKE ? HIGH : LOW);
        pwmDuty(pin, ch, 0);
    }
}

void motorsOff() {
    motorOut(true, 0);
    motorOut(false, 0);
}

void motorsBegin() {
    // rule: driver inputs low (coast; the output register resets to 0) as they become outputs, then PWM attached
    // at duty 0, then brake. (3.x digitalWrite refuses pins not yet set as GPIO, so pinMode comes first.)
    pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
    digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, LOW);
    digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, LOW);
    pwmSetup();
    lastOut[0] = lastOut[1] = 0x7fffffff;
    motorsOff();
}

}  // namespace hw
