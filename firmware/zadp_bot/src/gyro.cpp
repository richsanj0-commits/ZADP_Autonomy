// gyro.cpp : MPU6050 yaw-rate gyro, detected at boot on every robot. Same register setup as the micromouse firmware
// that was verified on robot 1 (D:\micromouse-Esp-build, initGyro / readGyroRaw): 0x68, +-500 deg/s, 44 Hz
// low-pass, the axis that feels gravity is the yaw axis.
// gyroBegin() runs in setup() with the robot still: probe (accelerometer read), bias = mean of GYRO_BIAS_SAMPLES
// readings, then it starts the gyro task. No gyro found = heading from the encoders (robots without one).
// rule: the control task never touches I2C. The gyro task reads the rate every GYRO_PERIOD_MS and publishes it
// through atomics; gyroRate() only copies it and rejects a value older than GYRO_STALE_MS.
// The 5 ToF XSHUT pins are held low (open drain, sink only) on every robot so ToF sensors, where fitted, stay in
// shutdown, off the bus. Harmless where nothing is connected: none of them is a strapping pin, and open drain
// never drives a pin high.
#include <Arduino.h>
#include <Wire.h>
#include <atomic>
#include <esp_timer.h>

#include "config.h"
#include "hw.h"

namespace hw {

static uint8_t axis = 2;                   // axis pointing up, found from gravity
static std::atomic<float> ccwSign{1.f};    // +1: raw + = counter-clockwise = heading +. From gravity at boot, then
                                           // checked against the encoders while turning (gyroCheckSign)
static float bias = 0.f;                   // raw reading when still
static std::atomic<bool> live{false};      // task running and readings valid
static std::atomic<float> rateRad{0.f};    // last yaw rate, rad/s, counter-clockwise +
static std::atomic<uint32_t> stampMs{0};   // when it was read
static std::atomic<const char*> why{"off (not probed)"};   // gyroStatus() text while not live
static uint8_t whoAmI = 0;                 // WHO_AM_I (0x75) register at boot, for the boot line
static const uint8_t TOF_XSHUT[] = {PIN_TOF_XSHUT_LIST};

static void wr(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(GYRO_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static bool rd(uint8_t reg, uint8_t* buf, uint8_t n) {
    Wire.beginTransmission(GYRO_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)GYRO_ADDR, (int)n) != n) return false;
    for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
    return true;
}

static bool raw(float& v) {
    uint8_t b[2];
    if (!rd(0x43 + 2 * axis, b, 2)) return false;
    v = (float)(int16_t)((b[0] << 8) | b[1]);
    return true;
}

static void gyroTask(void*) {
    TickType_t last = xTaskGetTickCount();
    uint16_t fails = 0;
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(GYRO_PERIOD_MS));
        if (!live.load()) vTaskDelete(nullptr);                  // rule: switched off by gyroCheckSign (dead chip): exit
        float v;
        if (raw(v)) {
            fails = 0;
            rateRad.store(ccwSign.load() * (v - bias) / GYRO_LSB_PER_DPS * 0.01745329252f);
            stampMs.store((uint32_t)(esp_timer_get_time() / 1000));
        } else if (++fails >= GYRO_MAX_FAILS) {                  // rule: a dead bus switches the gyro off for good
            why.store("off (lost: I2C reads failed)");
            live.store(false);
            vTaskDelete(nullptr);
        }
    }
}

bool gyroBegin() {
#if BENCH_SIM
    why.store("off (bench build: simulated wheels)");            // rule: bench build touches no I2C and no ToF pins
    return false;
#endif
    for (uint8_t p : TOF_XSHUT) {                                // rule: ToF sensors (if any) stay in shutdown
        pinMode(p, OUTPUT_OPEN_DRAIN);
        digitalWrite(p, LOW);
    }
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);
    Wire.setTimeOut(GYRO_I2C_TIMEOUT_MS);
    wr(0x6B, 0x00);             // wake up
    delay(100);
    wr(0x1A, 0x03);             // low-pass about 44 Hz
    wr(0x1B, 0x08);             // +-500 deg/s
    uint8_t a[6];
    rd(0x75, &whoAmI, 1);                                        // 0x68 on a genuine MPU6050 (clones differ)
    if (!rd(0x3B, a, 6)) {                                       // rule: no answer = no gyro, encoders only
        why.store("off (no MPU6050 at 0x68: heading from the encoders)");
        return false;
    }
    int16_t acc[3];
    for (int i = 0; i < 3; i++) acc[i] = (int16_t)((a[2 * i] << 8) | a[2 * i + 1]);
    axis = 0;
    for (int i = 1; i < 3; i++)
        if (abs(acc[i]) > abs(acc[axis])) axis = i;
    ccwSign.store(acc[axis] > 0 ? 1.f : -1.f);
    float sum = 0.f;
    int n = 0;
    for (int i = 0; i < GYRO_BIAS_SAMPLES; i++) {                // rule: the robot must be still at power-up
        float v;
        if (raw(v)) { sum += v; ++n; }
        delay(2);
    }
    if (n < GYRO_BIAS_SAMPLES / 2) {
        why.store("off (bias reads failed: heading from the encoders)");
        return false;
    }
    bias = sum / n;
    live.store(true);
    if (xTaskCreatePinnedToCore(gyroTask, "zadp_gyro", GYRO_TASK_STACK, nullptr, GYRO_TASK_PRIO, nullptr, GYRO_TASK_CORE) != pdPASS) {
        why.store("off (gyro task not created)");
        live.store(false);
        return false;
    }
    return true;
}

bool gyroOk() { return live.load(); }

uint8_t gyroWhoAmI() { return whoAmI; }

const char* gyroStatus() { return live.load() ? "on (heading rate from the gyro)" : why.load(); }

// rule (as in the micromouse firmware): the mounting sign is checked against the encoders. The wheels may slip,
// but they never report a turn in the opposite direction. Called by the control task every tick with the yaw
// rate from the encoders; after GYRO_SIGN_SAMPLES ticks of clear turning the gyro sign is flipped if it disagrees.
void gyroCheckSign(float encRate) {
    static float sum = 0.f, absSum = 0.f;
    static uint16_t n = 0;
    if (!live.load() || fabsf(encRate) < GYRO_SIGN_MIN_RATE) return;
    const float r = rateRad.load();
    sum += r * encRate;
    absSum += fabsf(r);
    if (++n < GYRO_SIGN_SAMPLES) return;
    if (absSum / n < GYRO_DEAD_RATE) {                           // rule: a chip that acks at 0x68 but does not follow
        why.store("off (gyro does not follow the turn)");        // the turn is no gyro: heading from the encoders
        live.store(false);                                       // (the gyro task sees live == false and exits)
    } else if (sum < 0.f) {
        ccwSign.store(-ccwSign.load());
        rateRad.store(-rateRad.load());
    }
    sum = 0.f; absSum = 0.f; n = 0;
}

// yaw rate in rad/s, counter-clockwise positive. false = no valid reading (caller uses the encoders).
bool gyroRate(float& radPerS) {
    if (!live.load()) return false;
    const uint32_t age = (uint32_t)(esp_timer_get_time() / 1000) - stampMs.load();
    if (age > GYRO_STALE_MS) return false;                       // rule: an old reading is not used
    radPerS = rateRad.load();
    return true;
}

}  // namespace hw
