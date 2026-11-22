// app.h : shared declarations of the FreeRTOS tasks of the ZADP truck firmware.
//
// Tasks (rates, priorities, cores, stacks: include/config.h, "FreeRTOS tasks"):
//   control   core 1, CTRL_TASK_PRIO, CONTROL_HZ (vTaskDelayUntil): encoders, zadp::Truck, motors, watchdogs
//   comms     core 0: WiFi + MQTT state machine, JSON parsing -> ctrlQ / camQ
//   telemetry core 0: TELEMETRY_HZ + at once on a new seq (notified by control) -> MQTT
//   console   core 1, low priority: serial bench self-test
//   health    core 1, lowest: status LED
// Data flow (no unprotected shared globals):
//   ctrlQ  (CMD_QUEUE_LEN, FIFO)    comms / console -> control: cmd, bench raw PWM, bench pose
//   camQ   (length 1, overwrite)    comms -> control: latest camera fix of this robot
//   snapQ  (length 1, overwrite)    control -> telemetry / console / health: consistent state snapshot (peek)
//   linkEvents (event group)        comms -> control / health / console: WiFi up, MQTT up
//   task notification               control -> telemetry: a new command was accepted (publish the ack now)
//   mqttMutex (inside comms_task)   comms <-> telemetry: the PubSubClient object is not thread safe
#pragma once

#include <Arduino.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "config.h"
#include "motion_core.h"

namespace app {

// ---------------------------------------------------------------- messages
enum CtrlKind : uint8_t { CTRL_CMD, CTRL_RAW, CTRL_POSE };
struct CtrlMsg {
    CtrlKind kind;
    zadp::Command cmd;           // CTRL_CMD
    float x, y, h;               // CTRL_POSE
    int rawL, rawR;              // CTRL_RAW
    float rawSec;                // CTRL_RAW (0 = end a raw test)
};
struct CamMsg {
    float x, y, h;
    double t;                    // image Unix time (camera PC clock; order check only), <= 0 unknown
    float age;                   // image age at publication, s (< 0: missing)
    int64_t rxUs;                // local arrival time (esp_timer_get_time): the queue wait is added to the lag
};

// written ONLY by the control task (xQueueOverwrite on snapQ), read by peeking
struct Snapshot {
    uint32_t seq = 0;
    int32_t encL = 0, encR = 0;
    float x = 0, y = 0, h = 0;
    zadp::State state = zadp::State::Idle;
    bool stalled = false, raw = false, camInit = false, linkStop = false;
    float pwmL = 0, pwmR = 0, vL = 0, vR = 0, refL = 0, refR = 0;
    uint32_t ticks = 0, overruns = 0, execMaxUs = 0;
    uint32_t camAccepted = 0, camRejected = 0, cmdNew = 0, cmdRepeat = 0;
};

// ---------------------------------------------------------------- RTOS objects (created in setup())
extern QueueHandle_t ctrlQ, camQ, snapQ;
extern EventGroupHandle_t linkEvents;
static const EventBits_t LINK_WIFI_UP = (1u << 0);
static const EventBits_t LINK_MQTT_UP = (1u << 1);

struct TaskInfo { const char* name; TaskHandle_t handle; uint32_t stack; };
enum TaskId { T_CTRL, T_COMMS, T_TEL, T_CONSOLE, T_HEALTH, T_COUNT };
extern TaskInfo tasks[T_COUNT];

// counters of the network side (atomic: written by one task, read by the console)
extern std::atomic<uint32_t> rxCmd, rxCam, badJson, ctrlDrops, telSent, telSkipped, mqttConnects;
// comms task heartbeat (millis() at the top of each pass): the control task treats a stuck comms task as link loss
extern std::atomic<uint32_t> commsAliveMs;

// ---------------------------------------------------------------- helpers
// rule: posting never blocks the caller (a full queue drops the message and counts it)
bool postCtrl(const CtrlMsg& m);
// latest snapshot; false only before the control task wrote the first one
bool getSnapshot(Snapshot& s);
inline bool mqttUp() { return (xEventGroupGetBits(linkEvents) & LINK_MQTT_UP) != 0; }
inline bool wifiUp() { return (xEventGroupGetBits(linkEvents) & LINK_WIFI_UP) != 0; }

// telemetry JSON of one snapshot (contract: {id, seq, encL, encR, x, y, heading, state}); length or -1
int formatTelemetry(const Snapshot& s, char* buf, size_t size);

// comms_task.cpp
void commsInit();                                               // mutex + client settings, before the tasks start
bool commsPublish(const char* topic, const char* payload, TickType_t wait);   // thread safe, QoS 0, not retained

// task bodies
void controlTask(void*);
void commsTask(void*);
void telemetryTask(void*);
void consoleTask(void*);
void healthTask(void*);
void printStackReport();

}  // namespace app
