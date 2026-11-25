// motors.cpp : TB6612FNG through LEDC PWM.
// arduino-esp32 3.x addresses PWM by pin (ledcAttach / ledcWrite(pin)); 2.x by channel
// (ledcSetup / ledcAttachPin / ledcWrite(channel)).
#include <Arduino.h>
#include <atomic>
#include <esp_timer.h>

#include "config.h"
#include "hw.h"

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2   // cores before 2.0 have no version header; they use the 2.x LEDC API
#endif

namespace hw {

// set by the motor guard after it forced the outputs to zero: the next motorOut() must write even if the value
// equals its cached one (the cache no longer shows what is on the pins)
static std::atomic<bool> guardRewrite[2];

#if BENCH_SIM
// ================================================================================================ bench build
// rule: no GPIO / LEDC pin is touched (the pins stay inputs). motorOut() only records the commanded signed PWM
// (same clamp as the real driver, logical direction: + = wheel forward); encoders.cpp integrates it.
static volatile int benchPwm[2] = {0, 0};
int benchWheelPwm(bool left) { return benchPwm[left ? 0 : 1]; }
void motorOut(bool left, int pwm) {
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    if (pwm < -PWM_MAX) pwm = -PWM_MAX;
    benchPwm[left ? 0 : 1] = pwm;
    guardRewrite[left ? 0 : 1] = false;                          // every call writes: nothing is cached here
}
void motorsOff() { motorOut(true, 0); motorOut(false, 0); }
void motorsBegin() { motorsOff(); }
// motor guard, bench build: zero the recorded PWM (the wheel model in encoders.cpp then runs down)
static void guardForceOff() { benchPwm[0] = 0; benchPwm[1] = 0; }

#else
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
    // rule: touch the hardware only on a change, or once after the motor guard overwrote the pins
    if (!guardRewrite[w].exchange(false) && pwm == lastOut[w]) return;
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

// motor guard, real build: both wheels to the zero-output state of motorOut(), written straight to the pins.
// rule: does not read or write lastOut (owned by the control task); task context only (esp_timer task, not an ISR).
static void guardForceOff() {
    digitalWrite(PIN_AIN1, ZERO_PWM_BRAKE ? HIGH : LOW); digitalWrite(PIN_AIN2, ZERO_PWM_BRAKE ? HIGH : LOW);
    digitalWrite(PIN_BIN1, ZERO_PWM_BRAKE ? HIGH : LOW); digitalWrite(PIN_BIN2, ZERO_PWM_BRAKE ? HIGH : LOW);
    pwmDuty(PIN_PWMA, LEDC_CH_L, 0);
    pwmDuty(PIN_PWMB, LEDC_CH_R, 0);
}

#endif  // BENCH_SIM

// ================================================================================================ motor guard
// Independent motor cut (real and bench builds). Only the control task writes the motors; if it hangs, the last PWM
// would stay on the pins until the task watchdog resets the chip (TWDT_TIMEOUT_MS). This periodic esp_timer callback
// runs in the esp_timer task (core 0, priority 22), so a control task stuck on core 1 cannot stop it.
static std::atomic<uint32_t> guardTicks(0), guardTrips(0);
static std::atomic<bool> guardCut(false);

void motorGuardKick() { ++guardTicks; }
uint32_t motorGuardTrips() { return guardTrips.load(); }
bool motorGuardCut() { return guardCut.load(); }

static void guardCallback(void*) {
    static uint32_t seen = 0;
    const int64_t us = esp_timer_get_time();
    // rule: the silence is counted from the first guard period, not from power-on (boot takes longer than
    // MOTOR_GUARD_MS and must not count as a trip before the control task had its first tick)
    static int64_t changedUs = us;
    const uint32_t t = guardTicks.load();
    if (t != seen) {                                             // the control task ticked: nothing to do
        seen = t; changedUs = us;
        guardCut = false;
        return;
    }
    // rule: no control tick for MOTOR_GUARD_MS = motor outputs forced to zero, again on every guard period while the
    // control task stays silent (a write the stuck task was in the middle of cannot outlive one period)
    if (us - changedUs < (int64_t)MOTOR_GUARD_MS * 1000) return;
    if (!guardCut.exchange(true)) ++guardTrips;
    guardForceOff();
    // rule: pins first, flags second: a control task that comes back always rewrites its value AFTER the last zero
    guardRewrite[0] = true; guardRewrite[1] = true;
}

void motorGuardBegin() {
    esp_timer_create_args_t args = {};
    args.callback = &guardCallback;
    args.dispatch_method = ESP_TIMER_TASK;                       // task context: GPIO and LEDC calls are allowed
    args.name = "zadp_mguard";
    esp_timer_handle_t h = nullptr;
    // rule: no guard = no start: ESP_ERROR_CHECK aborts, and the reset leaves the motor pins floating low (off)
    ESP_ERROR_CHECK(esp_timer_create(&args, &h));
    ESP_ERROR_CHECK(esp_timer_start_periodic(h, (uint64_t)MOTOR_GUARD_PERIOD_MS * 1000));
}

}  // namespace hw
