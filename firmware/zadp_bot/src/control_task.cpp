// control_task.cpp : the hard real-time task. Fixed CONTROL_HZ (vTaskDelayUntil), highest priority on core 1.
// Per tick: drain ctrlQ (commands, bench), take the latest camera fix (camQ), read the encoders, run zadp::Truck
// (odometry, fusion, state machine, laws, wheel PI; motion_core.h), apply the link-loss stop, write the motors,
// publish a snapshot (snapQ), notify telemetry on a new seq, feed the task watchdog.
// Rules: never blocks (all queue calls have timeout 0), no heap, no String, no network calls.
#include <Arduino.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>

#include "app.h"
#include "hw.h"

#if defined(CONFIG_FREERTOS_HZ) && (CONFIG_FREERTOS_HZ % CONTROL_HZ != 0)
#warning "FreeRTOS tick rate is not a multiple of CONTROL_HZ: the control period is rounded"
#endif

namespace app {

static zadp::Truck truck;   // owned by the control task: nobody else touches it

void controlTask(void*) {
    esp_task_wdt_add(nullptr);                                   // rule: the task watchdog resets the chip if we stall
    const TickType_t period = pdMS_TO_TICKS(1000 / CONTROL_HZ);
    const int64_t periodUs = 1000000LL / CONTROL_HZ;
    // link loss: MQTT down, or the comms task stuck, for LINK_LOSS_STOP_S = motors off (TOPICS.md watchdog 0.3 s);
    // bench commands exempt
    const uint32_t linkLossTicks = (uint32_t)(LINK_LOSS_STOP_S * CONTROL_HZ + 0.5f);
    const uint32_t commsStaleMs = (uint32_t)(LINK_LOSS_STOP_S * 1000.f + 0.5f);

    int32_t l = 0, r = 0;
    hw::encodersRead(l, r);
    truck.begin(zadp::makePose(INIT_X, INIT_Y, INIT_HEADING), l, r, esp_timer_get_time() * 1e-6);

    Snapshot s;
    uint32_t ticks = 0, overruns = 0, execMaxUs = 0, camAcc = 0, camRej = 0, cmdNew = 0, cmdRep = 0;
    uint32_t linkDown = 0;
    int64_t prevUs = esp_timer_get_time();
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, period);                          // rule: fixed rate, never waits on the network
        esp_task_wdt_reset();
        const int64_t us = esp_timer_get_time();
        if (us - prevUs > 2 * periodUs) ++overruns;              // a tick was missed
        prevUs = us;
        const double now = us * 1e-6;

        // ---- inputs (non-blocking)
        bool ack = false;
        CtrlMsg m;
        while (xQueueReceive(ctrlQ, &m, 0) == pdTRUE) {
            switch (m.kind) {
                case CTRL_CMD:
                    if (truck.onCommand(m.cmd, now)) { ++cmdNew; ack = true; }   // contract: ack telemetry on new seq
                    else ++cmdRep;
                    break;
                case CTRL_RAW:
                    truck.setRaw(m.rawL, m.rawR, m.rawSec, now);
                    break;
                case CTRL_POSE:
                    truck.setPose(zadp::makePose(m.x, m.y, m.h));
                    break;
            }
        }
        CamMsg c;
        if (xQueueReceive(camQ, &c, 0) == pdTRUE) {              // latest fix only (camQ is overwritten)
            // rule: lag = age at publication + time since arrival here (queue wait) + net delay (inside onCamera)
            const float age = c.age >= 0.f ? c.age + (float)((us - c.rxUs) * 1e-6) : c.age;
            zadp::CamResult cr = truck.onCamera(c.x, c.y, c.h, c.t, age, now);
            if (cr == zadp::CamResult::Accepted || cr == zadp::CamResult::Initialised || cr == zadp::CamResult::Relocked) ++camAcc;
            else ++camRej;
        }

        // ---- control
        hw::encodersRead(l, r);
        truck.step(now, l, r);

        // rule: MQTT down longer than LINK_LOSS_STOP_S = motors off, unless a bench command / raw test runs.
        // (The 300 ms command watchdog inside Truck already stops a drive or turn; this also covers braking
        // and anything else while the link is gone.)
        // rule: the link counts as up only while comms is alive too (fresh heartbeat), so a comms task blocked on
        // the MQTT lock or a socket cannot hide a dead link
        const uint32_t alive = commsAliveMs.load();
        const bool commsStale = (int32_t)((uint32_t)millis() - alive) > (int32_t)commsStaleMs;   // already 0.3 s old
        if (mqttUp()) linkDown = 0;
        else if (linkDown < 0xFFFFFFFFu) ++linkDown;
        const bool linkLost = linkDown > linkLossTicks || commsStale;
        const bool linkStop = linkLost && !truck.rawActive() && !truck.localActive();
        hw::motorOut(true, linkStop ? 0 : (int)lroundf(truck.pwmL()));
        hw::motorOut(false, linkStop ? 0 : (int)lroundf(truck.pwmR()));
        ++ticks;

        // ---- outputs
        s.seq = truck.seq(); s.encL = l; s.encR = r;
        s.x = truck.est().x; s.y = truck.est().y; s.h = truck.est().th;
        s.state = truck.state(); s.stalled = truck.stalled(); s.raw = truck.rawActive();
        s.camInit = truck.camInitialised(); s.linkStop = linkStop;
        s.pwmL = linkStop ? 0.f : truck.pwmL(); s.pwmR = linkStop ? 0.f : truck.pwmR();
        s.vL = truck.speedL(); s.vR = truck.speedR();
        s.refL = truck.refL(); s.refR = truck.refR();
        s.ticks = ticks; s.overruns = overruns;
        s.camAccepted = camAcc; s.camRejected = camRej; s.cmdNew = cmdNew; s.cmdRepeat = cmdRep;
        const uint32_t exec = (uint32_t)(esp_timer_get_time() - us);
        if (exec > execMaxUs) execMaxUs = exec;
        s.execMaxUs = execMaxUs;
        xQueueOverwrite(snapQ, &s);                              // rule: consistent snapshot, never blocks
        if (ack && tasks[T_TEL].handle) xTaskNotifyGive(tasks[T_TEL].handle);   // publish the ack now
    }
}

}  // namespace app
