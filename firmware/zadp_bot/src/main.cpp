// main.cpp : ZADP truck robot (ESP32 DevKit, TB6612FNG, two quadrature encoders), FreeRTOS application.
//
// setup() forces the motors off, starts the encoders, probes the gyro (starts its task if found), creates the
// queues / event group / mutex, configures the task watchdog and starts five pinned tasks (table in ../README.md,
// numbers in include/config.h):
//   control   core 1  prio CTRL_TASK_PRIO     CONTROL_HZ   encoders, odometry, fusion, laws, wheel PI, PWM, watchdogs
//   telemetry core 0  prio TEL_TASK_PRIO      TELEMETRY_HZ + ack on new seq
//   comms     core 0  prio COMMS_TASK_PRIO    WiFi + MQTT state machine, JSON -> queues, status on connect
//   console   core 1  prio CONSOLE_TASK_PRIO  serial bench self-test
//   health    core 1  prio HEALTH_TASK_PRIO   status LED
// The Arduino loop task is deleted: nothing runs as a polling loop.
// Contract: twin3d/link/TOPICS.md. Behaviour: twin3d/link/py/fake_bot.py. Maths: include/motion_core.h.
#include <Arduino.h>
#include <esp_idf_version.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

#include "app.h"
#include "hw.h"

namespace app {

QueueHandle_t ctrlQ = nullptr, camQ = nullptr, snapQ = nullptr;
EventGroupHandle_t linkEvents = nullptr;
TaskInfo tasks[T_COUNT] = {
    {"zadp_ctrl", nullptr, CTRL_TASK_STACK},
    {"zadp_comms", nullptr, COMMS_TASK_STACK},
    {"zadp_tel", nullptr, TEL_TASK_STACK},
    {"zadp_console", nullptr, CONSOLE_TASK_STACK},
    {"zadp_health", nullptr, HEALTH_TASK_STACK},
};
std::atomic<uint32_t> rxCmd(0), rxCam(0), badJson(0), ctrlDrops(0), telSent(0), telSkipped(0), mqttConnects(0);
std::atomic<uint32_t> commsAliveMs(0);

bool postCtrl(const CtrlMsg& m) {
    if (xQueueSend(ctrlQ, &m, 0) != pdTRUE) { ++ctrlDrops; return false; }   // rule: never block the sender
    return true;
}

bool getSnapshot(Snapshot& s) {
    if (xQueuePeek(snapQ, &s, 0) == pdTRUE) return true;
    s = Snapshot();
    return false;
}

const char* fwBuild() { return __DATE__ " " __TIME__; }

const char* resetReasonText() {
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON: return "poweron";
        case ESP_RST_EXT: return "external pin";
        case ESP_RST_SW: return "software";
        case ESP_RST_PANIC: return "panic";
        case ESP_RST_INT_WDT: return "interrupt watchdog";
        case ESP_RST_TASK_WDT: return "task watchdog";
        case ESP_RST_WDT: return "other watchdog";
        case ESP_RST_DEEPSLEEP: return "deep sleep";
        case ESP_RST_BROWNOUT: return "brownout";
        case ESP_RST_SDIO: return "sdio";
        case ESP_RST_UNKNOWN: return "unknown";
        default: return "other";                                 // IDF 5 adds USB, JTAG, eFuse, power glitch, lockup
    }
}

}  // namespace app

using namespace app;

// fatal setup error: motors stay off, the LED blinks fast forever, the message repeats on the serial port
static void halt(const char* why) {
    hw::motorsOff();
    for (;;) {
        Serial.printf("FATAL: %s\n", why);
        for (int i = 0; i < 20; ++i) { digitalWrite(PIN_LED, i & 1); delay(50); }
    }
}

// Task watchdog: ESP-IDF already runs it (Arduino sdkconfig); set our timeout, keep it panicking (= reset, which
// leaves every motor pin floating low = motors off). The control task subscribes itself.
// rule: the watchdog watches ONLY the control task. The Arduino default also watches the core-0 idle task, but
// PubSubClient busy-waits on core 0 (CONNACK wait, readByte) for up to MQTT_SOCKET_TIMEOUT_S > TWDT_TIMEOUT_MS,
// which would starve idle0 and reset the chip on a bad broker or a WiFi drop mid-packet.
static void twdtConfigure() {
#if ESP_IDF_VERSION_MAJOR >= 5
    esp_task_wdt_config_t cfg = {};
    cfg.timeout_ms = TWDT_TIMEOUT_MS;
    cfg.idle_core_mask = 0;                                      // no idle task: only the control task is watched
    cfg.trigger_panic = true;
    if (esp_task_wdt_reconfigure(&cfg) != ESP_OK) esp_task_wdt_init(&cfg);
#else
    esp_task_wdt_init((TWDT_TIMEOUT_MS + 999) / 1000, true);     // IDF 4.x: updates timeout / panic if already running
    disableCore0WDT();                                           // unsubscribe the core-0 idle task (esp32-hal-misc.c)
#endif
}

static void startTask(TaskId id, TaskFunction_t fn, UBaseType_t prio, BaseType_t core) {
    if (xTaskCreatePinnedToCore(fn, tasks[id].name, tasks[id].stack, nullptr, prio, &tasks[id].handle, core) != pdPASS)
        halt(tasks[id].name);
}

void setup() {
    hw::motorsBegin();                                           // rule: motors off before anything else
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);
    Serial.begin(SERIAL_BAUD);
    hw::encodersBegin();
    hw::gyroBegin();                                             // 0.9 s, robot still (bias); none found = encoders only
    Serial.printf("gyro: %s (WHO_AM_I 0x%02X)\n", hw::gyroStatus(), hw::gyroWhoAmI());

    ctrlQ = xQueueCreate(CMD_QUEUE_LEN, sizeof(CtrlMsg));
    camQ = xQueueCreate(CAM_QUEUE_LEN, sizeof(CamMsg));
    snapQ = xQueueCreate(1, sizeof(Snapshot));                   // length 1: xQueueOverwrite / xQueuePeek
    linkEvents = xEventGroupCreate();
    if (!ctrlQ || !camQ || !snapQ || !linkEvents) halt("out of memory creating the queues");
    commsInit();

    delay(200);
    Serial.printf("\nZADP truck bot %d, arduino-esp32 core %d.x, ESP-IDF %d.%d, encoders %s, cmd %s, telemetry %s\n", BOT_ID,
                  ESP_ARDUINO_VERSION_MAJOR, ESP_IDF_VERSION_MAJOR, ESP_IDF_VERSION_MINOR, hw::encoderBackend(), TOPIC_CMD,
                  TOPIC_TELEMETRY);
    Serial.printf("build %s, reset reason: %s, gyro: %s\n", fwBuild(), resetReasonText(), hw::gyroStatus());
    Serial.printf("FreeRTOS: control %d Hz (core %d prio %d), telemetry %d Hz, task watchdog %d ms, link-loss stop %.2f s\n",
                  CONTROL_HZ, CTRL_TASK_CORE, CTRL_TASK_PRIO, TELEMETRY_HZ, TWDT_TIMEOUT_MS, (double)LINK_LOSS_STOP_S);
    twdtConfigure();

    // telemetry first: the control task notifies it
    startTask(T_TEL, telemetryTask, TEL_TASK_PRIO, TEL_TASK_CORE);
    startTask(T_CTRL, controlTask, CTRL_TASK_PRIO, CTRL_TASK_CORE);
    startTask(T_COMMS, commsTask, COMMS_TASK_PRIO, COMMS_TASK_CORE);
    startTask(T_CONSOLE, consoleTask, CONSOLE_TASK_PRIO, CONSOLE_TASK_CORE);
    startTask(T_HEALTH, healthTask, HEALTH_TASK_PRIO, HEALTH_TASK_CORE);
}

void loop() {
    vTaskDelete(nullptr);                                        // everything runs in the tasks above
}
