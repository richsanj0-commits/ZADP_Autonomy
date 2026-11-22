// encoders.cpp : two quadrature encoders, 4x decoding, forward positive.
//
// ENC_USE_PCNT 1 (default): ESP32 pulse counter hardware, one unit per wheel, two channels per unit (A edges
// gated by B, B edges gated by A) = every edge of A and B counts, like the interrupt decoder. Glitch filter
// PCNT_FILTER_NS. The 16-bit hardware counter resets to 0 at +-PCNT_LIMIT; encodersRead() (control task, every
// tick, at most ~70 counts per tick at top speed) unwraps it into a 32-bit count.
//   arduino-esp32 2.x (ESP-IDF 4.4): legacy driver/pcnt.h, unwrap in software.
//   arduino-esp32 3.x (ESP-IDF 5.x): driver/pulse_cnt.h with accum_count (the driver unwraps at the limits).
// ENC_USE_PCNT 0: the previous GPIO interrupt decoder (IRAM ISR, state table, atomic read).
//
// Sign convention of both decoders (same table): A rising while B low = +1, B rising while A low = -1, then
// multiplied by ENC_x_DIR.
#include <Arduino.h>
#include <esp_idf_version.h>

#include "config.h"
#include "hw.h"

#if ENC_USE_PCNT
#if ESP_IDF_VERSION_MAJOR >= 5
#include <driver/pulse_cnt.h>
#else
#include <driver/pcnt.h>
#endif
#else
#include <soc/gpio_reg.h>
#include <soc/soc.h>
#endif

#if PCNT_LIMIT > 32767 || PCNT_LIMIT < 1000
#error "PCNT_LIMIT must fit the 16-bit counter and leave room for one tick of counts"
#endif

namespace hw {

#if ENC_USE_PCNT
// ================================================================================================ PCNT
static const int ENC_PIN_A[2] = {PIN_ENC_L_A, PIN_ENC_R_A};
static const int ENC_PIN_B[2] = {PIN_ENC_L_B, PIN_ENC_R_B};
static const int ENC_SIGN[2] = {ENC_L_DIR, ENC_R_DIR};

#if ESP_IDF_VERSION_MAJOR >= 5
static pcnt_unit_handle_t unitH[2] = {nullptr, nullptr};

static void setupUnit(int w) {
    pcnt_unit_config_t uc = {};
    uc.low_limit = -PCNT_LIMIT;
    uc.high_limit = PCNT_LIMIT;
    uc.flags.accum_count = 1;                                    // rule: driver adds the limit on each overflow
    ESP_ERROR_CHECK(pcnt_new_unit(&uc, &unitH[w]));
    pcnt_glitch_filter_config_t fc = {};
    fc.max_glitch_ns = PCNT_FILTER_NS;
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unitH[w], &fc));
    pcnt_chan_config_t ca = {};
    ca.edge_gpio_num = ENC_PIN_A[w];
    ca.level_gpio_num = ENC_PIN_B[w];
    pcnt_channel_handle_t chA = nullptr, chB = nullptr;
    ESP_ERROR_CHECK(pcnt_new_channel(unitH[w], &ca, &chA));
    pcnt_chan_config_t cb = {};
    cb.edge_gpio_num = ENC_PIN_B[w];
    cb.level_gpio_num = ENC_PIN_A[w];
    ESP_ERROR_CHECK(pcnt_new_channel(unitH[w], &cb, &chB));
    // channel A: A rising = +1 with B low, -1 with B high (B high inverts); A falling the opposite
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chA, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chA, PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP));
    // channel B: B rising = -1 with A low, +1 with A high; B falling the opposite
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chB, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chB, PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unitH[w], PCNT_LIMIT));     // needed by accum_count
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unitH[w], -PCNT_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(unitH[w]));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unitH[w]));
    ESP_ERROR_CHECK(pcnt_unit_start(unitH[w]));
}

static int32_t readUnit(int w) {
    int v = 0;
    pcnt_unit_get_count(unitH[w], &v);                           // accumulated count (int)
    return (int32_t)v;
}
#else
static const pcnt_unit_t UNIT[2] = {PCNT_UNIT_0, PCNT_UNIT_1};
static int16_t lastRaw[2] = {0, 0};
static int32_t acc[2] = {0, 0};

static void setupUnit(int w) {
    pcnt_config_t c = {};
    c.unit = UNIT[w];
    c.counter_h_lim = PCNT_LIMIT;
    c.counter_l_lim = -PCNT_LIMIT;
    // channel 0: A edges, B level. A rising = +1 with B low, inverted with B high.
    c.channel = PCNT_CHANNEL_0;
    c.pulse_gpio_num = ENC_PIN_A[w];
    c.ctrl_gpio_num = ENC_PIN_B[w];
    c.pos_mode = PCNT_COUNT_INC;
    c.neg_mode = PCNT_COUNT_DEC;
    c.hctrl_mode = PCNT_MODE_REVERSE;
    c.lctrl_mode = PCNT_MODE_KEEP;
    ESP_ERROR_CHECK(pcnt_unit_config(&c));
    // channel 1: B edges, A level. B rising = -1 with A low, inverted with A high.
    c.channel = PCNT_CHANNEL_1;
    c.pulse_gpio_num = ENC_PIN_B[w];
    c.ctrl_gpio_num = ENC_PIN_A[w];
    c.pos_mode = PCNT_COUNT_DEC;
    c.neg_mode = PCNT_COUNT_INC;
    c.hctrl_mode = PCNT_MODE_REVERSE;
    c.lctrl_mode = PCNT_MODE_KEEP;
    ESP_ERROR_CHECK(pcnt_unit_config(&c));
    // filter value in APB cycles (80 MHz), 10 bit
    uint32_t f = (uint32_t)PCNT_FILTER_NS * 80u / 1000u;
    if (f > 1023u) f = 1023u;
    ESP_ERROR_CHECK(pcnt_set_filter_value(UNIT[w], (uint16_t)f));
    ESP_ERROR_CHECK(pcnt_filter_enable(UNIT[w]));
    // rule: limit comparators on, so the counter resets to 0 at +-PCNT_LIMIT (pcnt_unit_config disables them);
    // no interrupt is installed, the unwrap below needs only the reset
    ESP_ERROR_CHECK(pcnt_event_enable(UNIT[w], PCNT_EVT_H_LIM));
    ESP_ERROR_CHECK(pcnt_event_enable(UNIT[w], PCNT_EVT_L_LIM));
    ESP_ERROR_CHECK(pcnt_counter_pause(UNIT[w]));
    ESP_ERROR_CHECK(pcnt_counter_clear(UNIT[w]));
    ESP_ERROR_CHECK(pcnt_counter_resume(UNIT[w]));
    lastRaw[w] = 0;
    acc[w] = 0;
}

static int32_t readUnit(int w) {
    int16_t raw = 0;
    pcnt_get_counter_value(UNIT[w], &raw);
    int32_t d = (int32_t)raw - lastRaw[w];
    // rule: unwrap. The counter resets to 0 at +-PCNT_LIMIT, so it counts modulo PCNT_LIMIT; a change of more
    // than half the range in one 5 ms tick is impossible (max ~70 counts), so it is a wrap
    if (d > PCNT_LIMIT / 2) d -= PCNT_LIMIT;
    else if (d < -PCNT_LIMIT / 2) d += PCNT_LIMIT;
    lastRaw[w] = raw;
    acc[w] += d;
    return acc[w];
}
#endif

void encodersBegin() {
    pinMode(PIN_ENC_L_A, INPUT); pinMode(PIN_ENC_L_B, INPUT);   // 34..39: input only, external pull-ups
    pinMode(PIN_ENC_R_A, INPUT); pinMode(PIN_ENC_R_B, INPUT);
    setupUnit(0);
    setupUnit(1);
}

void encodersRead(int32_t& l, int32_t& r) {
    l = readUnit(0) * ENC_SIGN[0];
    r = readUnit(1) * ENC_SIGN[1];
}

const char* encoderBackend() { return "pcnt"; }

#else
// ================================================================================================ GPIO ISR
// Every edge of A or B interrupts; the ISR reads both pins and looks up the step. A spurious interrupt (no pin
// change, e.g. the GPIO 36/39 glitch of the ESP32) reads the same state and adds 0.
static portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;
static volatile int32_t encCount[2] = {0, 0};
static volatile uint8_t encState[2] = {0, 0};
static const int8_t DRAM_ATTR QDEC[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

static inline uint32_t IRAM_ATTR pinLevel(uint8_t pin) {
    return pin < 32 ? (REG_READ(GPIO_IN_REG) >> pin) & 1u : (REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 1u;
}
static void IRAM_ATTR isrEncL() {
    uint8_t s = (uint8_t)((pinLevel(PIN_ENC_L_A) << 1) | pinLevel(PIN_ENC_L_B));
    portENTER_CRITICAL_ISR(&encMux);
    encCount[0] += QDEC[(encState[0] << 2) | s] * ENC_L_DIR;
    encState[0] = s;
    portEXIT_CRITICAL_ISR(&encMux);
}
static void IRAM_ATTR isrEncR() {
    uint8_t s = (uint8_t)((pinLevel(PIN_ENC_R_A) << 1) | pinLevel(PIN_ENC_R_B));
    portENTER_CRITICAL_ISR(&encMux);
    encCount[1] += QDEC[(encState[1] << 2) | s] * ENC_R_DIR;
    encState[1] = s;
    portEXIT_CRITICAL_ISR(&encMux);
}

void encodersBegin() {
    pinMode(PIN_ENC_L_A, INPUT); pinMode(PIN_ENC_L_B, INPUT);
    pinMode(PIN_ENC_R_A, INPUT); pinMode(PIN_ENC_R_B, INPUT);
    encState[0] = (uint8_t)((pinLevel(PIN_ENC_L_A) << 1) | pinLevel(PIN_ENC_L_B));
    encState[1] = (uint8_t)((pinLevel(PIN_ENC_R_A) << 1) | pinLevel(PIN_ENC_R_B));
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_B), isrEncL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_B), isrEncR, CHANGE);
}

// rule: both counters read together, atomically
void encodersRead(int32_t& l, int32_t& r) {
    portENTER_CRITICAL(&encMux);
    l = encCount[0];
    r = encCount[1];
    portEXIT_CRITICAL(&encMux);
}

const char* encoderBackend() { return "gpio-isr"; }
#endif

}  // namespace hw
