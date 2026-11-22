// telemetry_task.cpp : publishes zadp/botN/telemetry at TELEMETRY_HZ, and at once when the control task notifies a
// newly accepted command (the brain's acknowledgement; the next periodic message follows one period later).
// Reads the state only from snapQ (consistent snapshot written by the control task).
#include <Arduino.h>

#include "app.h"

namespace app {

int formatTelemetry(const Snapshot& s, char* buf, size_t size) {
    // contract: {id, seq, encL, encR, x, y, heading, state}
    int n = snprintf(buf, size,
                     "{\"id\":%d,\"seq\":%lu,\"encL\":%ld,\"encR\":%ld,\"x\":%.5f,\"y\":%.5f,\"heading\":%.5f,\"state\":\"%s\"}",
                     BOT_ID, (unsigned long)s.seq, (long)s.encL, (long)s.encR, (double)s.x, (double)s.y, (double)s.h,
                     zadp::stateName(s.state));
    return (n > 0 && n < (int)size) ? n : -1;
}

static void publishNow(char* buf) {
    if (!mqttUp()) return;
    Snapshot s;
    if (!getSnapshot(s) || formatTelemetry(s, buf, TEL_BUF_SIZE) < 0) return;
    if (commsPublish(TOPIC_TELEMETRY, buf, pdMS_TO_TICKS(TEL_MUTEX_WAIT_MS))) ++telSent;
    else ++telSkipped;                                           // client busy (connecting) or publish failed
}

void telemetryTask(void*) {
    static char buf[TEL_BUF_SIZE];
    const TickType_t period = pdMS_TO_TICKS(1000 / TELEMETRY_HZ);
    TickType_t next = xTaskGetTickCount() + period;
    for (;;) {
        TickType_t now = xTaskGetTickCount();
        TickType_t wait = (int32_t)(next - now) > 0 ? next - now : 0;
        const bool ack = ulTaskNotifyTake(pdTRUE, wait) > 0;     // woken by a new seq or by the period
        now = xTaskGetTickCount();
        const bool due = (int32_t)(now - next) >= 0;
        if (!ack && !due) continue;
        if (ack) next = now + period;                            // rule: the ack restarts the period
        else {
            next += period;
            if ((int32_t)(now - next) >= 0) next = now + period; // fell behind: resync, no burst
        }
        publishNow(buf);
    }
}

}  // namespace app
