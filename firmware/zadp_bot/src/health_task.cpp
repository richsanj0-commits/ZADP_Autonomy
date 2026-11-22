// health_task.cpp : status LED (lowest priority, core 1). No printf here (small stack): the boot stack report
// runs in the console task.
// no WiFi: 5 Hz blink | WiFi but no broker: double blink each second | watchdog stop: 10 Hz flicker
// moving / turning / raw test: on | dumping: 2 Hz | idle / arrived: short flash every 2 s
#include <Arduino.h>

#include "app.h"

namespace app {

void healthTask(void*) {
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(HEALTH_PERIOD_MS));
        const uint32_t ms = millis();
        Snapshot s;
        getSnapshot(s);
        bool on;
        if (s.raw) on = true;
        else if (!wifiUp()) on = (ms / 100) % 2;
        else if (!mqttUp()) { uint32_t p = ms % 1000; on = p < 100 || (p >= 200 && p < 300); }
        else if (s.stalled) on = (ms / 50) % 2;
        else if (s.state == zadp::State::Moving || s.state == zadp::State::Turning) on = true;
        else if (s.state == zadp::State::Dumping) on = (ms / 250) % 2;
        else on = (ms % 2000) < 80;
        digitalWrite(PIN_LED, on ? HIGH : LOW);
    }
}

}  // namespace app
