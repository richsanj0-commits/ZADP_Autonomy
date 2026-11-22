// comms_task.cpp : WiFi + MQTT on core 0. Owns the PubSubClient object (guarded by mqttMutex because the
// telemetry task publishes through it). Incoming messages are parsed here and handed to the control task:
//   cmd      -> ctrlQ (FIFO, xQueueSend timeout 0)
//   cam/pose -> camQ  (length 1, xQueueOverwrite: latest wins)
// Connection state machine (non-blocking for everything but this task; a broker connect attempt may block THIS
// task for up to the TCP connect timeout):
//   WIFI_DOWN --WiFi up--> MQTT_DOWN --connect ok--> MQTT_UP --socket lost--> MQTT_DOWN --WiFi lost--> WIFI_DOWN
// Retries back off exponentially: WiFi WIFI_RETRY_MS .. WIFI_RETRY_MAX_MS, broker MQTT_RETRY_MS .. MQTT_RETRY_MAX_MS.
// Contract: twin3d/link/TOPICS.md (QoS, clean session, keep-alive 15 s, last will offline).
#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <esp_timer.h>
#include <freertos/semphr.h>

#include "app.h"

#if __has_include("secrets.h")
#include "secrets.h"
#if !defined(WIFI_SSID) || !defined(WIFI_PASS) || !defined(MQTT_HOST)
#error "include/secrets.h must define WIFI_SSID, WIFI_PASS and MQTT_HOST (see include/secrets.example.h)"
#endif
#else
#error "include/secrets.h is missing: copy include/secrets.example.h to include/secrets.h and set WIFI_SSID, WIFI_PASS, MQTT_HOST"
#endif
#ifndef MQTT_USER
#define MQTT_USER nullptr
#endif
#ifndef MQTT_PASS
#define MQTT_PASS nullptr
#endif

namespace app {

static WiFiClient net;
static PubSubClient mqtt(net);
static SemaphoreHandle_t mqttMutex = nullptr;
static const char* WILL_MSG = "{\"id\":" ZADP_STR(BOT_ID) ",\"state\":\"offline\"}";

// ---------------------------------------------------------------- parsing (runs inside mqtt.loop())
static bool numField(JsonVariantConst v, float& out) {
    if (v.is<float>() || v.is<long>()) { out = v.as<float>(); return true; }
    return false;
}

static void handleCmd(const uint8_t* payload, unsigned int len) {
    JsonDocument doc;
    if (deserializeJson(doc, payload, len)) { ++badJson; return; }
    JsonVariantConst id = doc["id"];
    if (!id.isNull() && id.as<int>() != BOT_ID) return;          // rule: only commands for this robot
    CtrlMsg m{};
    m.kind = CTRL_CMD;
    zadp::Command& c = m.cmd;
    c.seq = doc["seq"].as<uint32_t>();                           // contract: uint32; missing = 0 (first-draft message)
    const char* a = doc["action"] | "";
    if (!strcmp(a, "goto")) c.action = zadp::Action::Goto;
    else if (!strcmp(a, "turn")) c.action = zadp::Action::Turn;
    else if (!strcmp(a, "dump")) c.action = zadp::Action::Dump;
    else if (!strcmp(a, "stop")) c.action = zadp::Action::Stop;
    else c.action = zadp::Action::Unknown;                       // rule: unknown action = ignore, report idle
    c.hasXY = numField(doc["x"], c.x) && numField(doc["y"], c.y);
    c.hasHeading = numField(doc["heading"], c.heading);
    float sp = 0.f;
    c.speed = numField(doc["speed"], sp) ? sp : 0.f;             // <= 0 / missing = default cruise
    c.dir = doc["dir"].as<int>();                                // missing = 0 = shortest
    float tol = 0.f;
    c.tolDeg = numField(doc["tol"], tol) ? tol : 0.f;            // turn: tolerance in deg; missing = HEADING_TOL_DEG
    c.local = false;
    postCtrl(m);
    ++rxCmd;
}

static void handleCam(const uint8_t* payload, unsigned int len) {
    JsonDocument doc;
    if (deserializeJson(doc, payload, len)) { ++badJson; return; }
    if (doc["bot"].as<int>() != BOT_ID) return;                  // rule: use only this robot's pose
    CamMsg m{};
    if (!numField(doc["x"], m.x) || !numField(doc["y"], m.y) || !numField(doc["heading"], m.h)) return;
    m.t = doc["t"].as<double>();                                 // needs ARDUINOJSON_USE_DOUBLE (platformio.ini)
    float age = -1.f;
    m.age = numField(doc["age"], age) ? age : -1.f;              // contract: age = image age at publication, s
    m.rxUs = esp_timer_get_time();                               // arrival: the control task adds the queue wait
    xQueueOverwrite(camQ, &m);                                   // rule: latest fix wins, never blocks
    ++rxCam;
}

static void onMqtt(char* topic, uint8_t* payload, unsigned int len) {
    if (!strcmp(topic, TOPIC_CMD)) handleCmd(payload, len);
    else if (!strcmp(topic, TOPIC_CAM_POSE)) handleCam(payload, len);
}

// ---------------------------------------------------------------- publishing
// caller holds mqttMutex
static bool publishTelemetryLocked() {
    Snapshot s;
    char buf[TEL_BUF_SIZE];
    if (!getSnapshot(s)) return false;
    int n = formatTelemetry(s, buf, sizeof buf);
    return n > 0 && mqtt.publish(TOPIC_TELEMETRY, buf, false);
}

bool commsPublish(const char* topic, const char* payload, TickType_t wait) {
    if (!mqttMutex || xSemaphoreTake(mqttMutex, wait) != pdTRUE) return false;
    bool ok = mqtt.connected() && mqtt.publish(topic, payload, false);   // contract: QoS 0, not retained
    xSemaphoreGive(mqttMutex);
    return ok;
}

void commsInit() {
    mqttMutex = xSemaphoreCreateMutex();                         // priority inheritance (telemetry > comms)
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setCallback(onMqtt);
    mqtt.setBufferSize(MQTT_BUFFER_SIZE);
    mqtt.setKeepAlive(MQTT_KEEPALIVE_S);
    mqtt.setSocketTimeout(MQTT_SOCKET_TIMEOUT_S);
}

static uint32_t backoff(uint32_t cur, uint32_t first, uint32_t maxMs) {
    if (cur < first) return first;
    return cur >= maxMs / 2 ? maxMs : cur * 2;
}

// ---------------------------------------------------------------- task
void commsTask(void*) {
    WiFi.persistent(false);                                      // no NVS flash writes (they stall core 1 too)
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);                                        // no modem sleep: lower command latency
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);                            // returns at once
    uint32_t wifiTry = millis(), wifiWait = WIFI_RETRY_MS;
    uint32_t mqttTry = 0, mqttWait = 0;                          // first broker attempt as soon as WiFi is up
    bool wifiWasUp = false, mqttWasUp = false;

    for (;;) {
        const uint32_t ms = millis();
        commsAliveMs.store(ms);                                  // heartbeat: goes stale while this task is stuck
        // ---------------- WIFI_DOWN
        if (WiFi.status() != WL_CONNECTED) {
            // rule: clear the link bits FIRST, never behind the mutex (telemetry may hold it in a blocked write)
            xEventGroupClearBits(linkEvents, LINK_WIFI_UP | LINK_MQTT_UP);
            if (wifiWasUp) {
                Serial.println("[wifi] lost");
                wifiWasUp = false;
                wifiTry = ms;
                wifiWait = WIFI_RETRY_MS;
            }
            if (mqttWasUp) {
                xSemaphoreTake(mqttMutex, portMAX_DELAY);
                mqtt.disconnect();                               // drop the dead socket
                xSemaphoreGive(mqttMutex);
                mqttWasUp = false;
            }
            if (ms - wifiTry >= wifiWait) {
                wifiTry = ms;
                Serial.printf("[wifi] still down, reconnect (next try in %lu ms)\n", (unsigned long)backoff(wifiWait, WIFI_RETRY_MS, WIFI_RETRY_MAX_MS));
                WiFi.reconnect();
                wifiWait = backoff(wifiWait, WIFI_RETRY_MS, WIFI_RETRY_MAX_MS);
            }
            vTaskDelay(pdMS_TO_TICKS(COMMS_IDLE_MS));
            continue;
        }
        if (!wifiWasUp) {
            wifiWasUp = true;
            wifiWait = WIFI_RETRY_MS;
            mqttTry = ms; mqttWait = 0;
            xEventGroupSetBits(linkEvents, LINK_WIFI_UP);
            Serial.printf("[wifi] connected, ip %s, rssi %d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
        }

        xSemaphoreTake(mqttMutex, portMAX_DELAY);
        if (mqtt.connected()) {
            // ---------------- MQTT_UP
            mqtt.loop();                                         // receives cmd / cam (callbacks above), keep-alive
            // PubSubClient reads ONE packet per loop(): drain a camera burst / queued cmd now (bounded)
            for (int i = 0; i < COMMS_DRAIN_MAX && mqtt.connected() && net.available() > 0; ++i) mqtt.loop();
        } else {
            // ---------------- MQTT_DOWN
            if (mqttWasUp) {
                mqttWasUp = false;
                xEventGroupClearBits(linkEvents, LINK_MQTT_UP);
                Serial.printf("[mqtt] connection lost, state %d\n", mqtt.state());
                mqttTry = ms; mqttWait = MQTT_RETRY_MS;
            }
            if (ms - mqttTry >= mqttWait) {
                mqttTry = ms;
                Serial.printf("[mqtt] connecting to %s:%d as %s ... ", MQTT_HOST, MQTT_PORT, MQTT_CLIENT_ID);
                // contract: clean session; last will {"id":N,"state":"offline"} on the telemetry topic, QoS 0, not retained
                if (mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS, TOPIC_TELEMETRY, 0, false, WILL_MSG, true)) {
                    bool ok1 = mqtt.subscribe(TOPIC_CMD, 1);     // contract: cmd at QoS 1
                    bool ok2 = mqtt.subscribe(TOPIC_CAM_POSE, 0);    // contract: cam/pose at QoS 0
                    if (!ok1 || !ok2) {
                        // rule: never report the link up without both subscriptions: drop it and retry
                        mqtt.disconnect();
                        mqttWait = backoff(mqttWait, MQTT_RETRY_MS, MQTT_RETRY_MAX_MS);
                        Serial.printf("subscribe failed (cmd %d, cam %d), next try in %lu ms\n", ok1, ok2, (unsigned long)mqttWait);
                    } else {
                        Serial.println("ok (subscribed cmd, cam/pose)");
                        mqttWasUp = true;
                        mqttWait = MQTT_RETRY_MS;
                        ++mqttConnects;
                        xEventGroupSetBits(linkEvents, LINK_MQTT_UP);
                        if (publishTelemetryLocked()) ++telSent; // announce the robot at once
                    }
                } else {
                    mqttWait = backoff(mqttWait, MQTT_RETRY_MS, MQTT_RETRY_MAX_MS);
                    Serial.printf("failed, state %d, next try in %lu ms\n", mqtt.state(), (unsigned long)mqttWait);
                }
            }
        }
        xSemaphoreGive(mqttMutex);
        vTaskDelay(pdMS_TO_TICKS(COMMS_PERIOD_MS));
    }
}

}  // namespace app
