// console_task.cpp : serial bench self-test (115200 baud), low priority on core 1. Commands go to the control task
// through ctrlQ (marked local = exempt from the 300 ms watchdog and from the link-loss stop); state is read from
// snapQ. No String, fixed line buffer. Also prints the stack report once, BOOT_STACK_REPORT_MS after boot.
#include <Arduino.h>

#include "app.h"
#include "hw.h"

namespace app {

static bool statusStream = false;
static int32_t markL = 0, markR = 0;
static uint32_t localSeq = 0xF0000000u;   // bench seqs: far from the brain's (time base mod 2^30)

static void printHelp() {
    Serial.println(F(
        "ZADP truck self-test. Commands (Enter to send):\n"
        "  h            this help\n"
        "  l [pwm] [s]  LEFT wheel forward, open loop (default pwm 120 for 2 s)\n"
        "  r [pwm] [s]  RIGHT wheel forward, open loop\n"
        "  b [pwm] [s]  both wheels forward, open loop\n"
        "  e            print encoder counts (and counts / revolutions since 'm')\n"
        "  m            mark the encoder counts now (for the counts-per-revolution check)\n"
        "  d [m]        drive forward 0.2 m (closed loop, on the estimate)\n"
        "  t [deg]      turn in place +90 deg (counter-clockwise; negative = clockwise)\n"
        "  s            stop\n"
        "  p x y deg    set the pose estimate\n"
        "  i            status once;  v  status every second on/off\n"
        "  w            FreeRTOS tasks: unused stack (high-water mark), free heap\n"
        "Bench commands are exempt from the 300 ms watchdog. A command from the broker replaces them."));
}

void printStackReport() {
    Serial.printf("[tasks] %-10s %5s %6s %8s\n", "name", "core", "stack", "unused");
    static const int CORE[T_COUNT] = {CTRL_TASK_CORE, COMMS_TASK_CORE, TEL_TASK_CORE, CONSOLE_TASK_CORE, HEALTH_TASK_CORE};
    for (int i = 0; i < T_COUNT; ++i) {
        if (!tasks[i].handle) continue;
        // ESP-IDF: stack depth and high-water mark are in bytes
        Serial.printf("[tasks] %-10s %5d %6lu %8lu%s\n", tasks[i].name, CORE[i], (unsigned long)tasks[i].stack,
                      (unsigned long)uxTaskGetStackHighWaterMark(tasks[i].handle),
                      uxTaskGetStackHighWaterMark(tasks[i].handle) < 512 ? "  LOW: raise the stack in config.h" : "");
    }
    Serial.printf("[tasks] free heap %lu, min free heap %lu\n", (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMinFreeHeap());
}

static void printStatus() {
    Snapshot s;
    getSnapshot(s);
    Serial.printf("[%lu ms] bot %d seq %lu state %s%s%s%s | pose %.4f %.4f %.1f deg | enc %ld %ld | v %.3f %.3f ref %.3f %.3f "
                  "pwm %.0f %.0f | wifi %d mqtt %d (connects %lu) | rx cmd %lu (new %lu rep %lu) cam %lu (acc %lu rej %lu) "
                  "tel %lu (skipped %lu) | ticks %lu overruns %lu exec max %lu us drops %lu badjson %lu\n",
                  (unsigned long)millis(), BOT_ID, (unsigned long)s.seq, zadp::stateName(s.state), s.stalled ? " (watchdog)" : "",
                  s.raw ? " (raw)" : "", s.linkStop ? " (link lost: motors off)" : "", (double)s.x, (double)s.y,
                  (double)(s.h * 180.f / zadp::kPi), (long)s.encL, (long)s.encR, (double)s.vL, (double)s.vR, (double)s.refL,
                  (double)s.refR, (double)s.pwmL, (double)s.pwmR, (int)wifiUp(), (int)mqttUp(),
                  (unsigned long)mqttConnects.load(), (unsigned long)rxCmd.load(), (unsigned long)s.cmdNew,
                  (unsigned long)s.cmdRepeat, (unsigned long)rxCam.load(), (unsigned long)s.camAccepted,
                  (unsigned long)s.camRejected, (unsigned long)telSent.load(), (unsigned long)telSkipped.load(),
                  (unsigned long)s.ticks, (unsigned long)s.overruns, (unsigned long)s.execMaxUs,
                  (unsigned long)ctrlDrops.load(), (unsigned long)badJson.load());
}

static bool localCommand(zadp::Action a, float x, float y, float h, int dir) {
    CtrlMsg m{};
    m.kind = CTRL_CMD;
    m.cmd.seq = ++localSeq;
    m.cmd.action = a;
    m.cmd.x = x; m.cmd.y = y; m.cmd.heading = h;
    m.cmd.hasXY = true; m.cmd.hasHeading = true;
    m.cmd.dir = dir;
    m.cmd.local = true;
    return postCtrl(m);
}

static void runLine(char* line) {
    while (*line == ' ' || *line == '\t') ++line;               // trim
    size_t n = strlen(line);
    while (n && (line[n - 1] == ' ' || line[n - 1] == '\t')) line[--n] = 0;
    if (!n) return;
    const char c = line[0];
    float a1 = NAN, a2 = NAN, a3 = NAN;
    sscanf(line + 1, "%f %f %f", &a1, &a2, &a3);
    Snapshot sn;
    getSnapshot(sn);
    switch (c) {
        case 'h': case '?': printHelp(); break;
        case 'l': case 'r': case 'b': {
            // rule: bench raw test bounded: |pwm| <= PWM_MAX, duration 0..RAW_MAX_S
            int pwm = isnan(a1) ? 120 : constrain((int)a1, -PWM_MAX, PWM_MAX);
            float sec = isnan(a2) ? 2.f : constrain(a2, 0.f, RAW_MAX_S);
            CtrlMsg m{};
            m.kind = CTRL_RAW;
            m.rawL = (c == 'l' || c == 'b') ? pwm : 0;
            m.rawR = (c == 'r' || c == 'b') ? pwm : 0;
            m.rawSec = sec;
            postCtrl(m);
            Serial.printf("raw pwm L %d R %d for %.1f s: the wheel(s) must turn FORWARD and the counts go UP\n", m.rawL, m.rawR,
                          (double)sec);
            break;
        }
        case 'e': {
            const int32_t l = sn.encL, r = sn.encR;                 // counts of the last control tick (<= 5 ms old)
            Serial.printf("enc L %ld R %ld | since mark: L %ld (%.3f rev) R %ld (%.3f rev) | CPR L %.1f R %.1f\n", (long)l, (long)r,
                          (long)(l - markL), (double)((l - markL) / CPR_LEFT), (long)(r - markR),
                          (double)((r - markR) / CPR_RIGHT), (double)CPR_LEFT, (double)CPR_RIGHT);
            break;
        }
        case 'm':
            markL = sn.encL; markR = sn.encR;
            Serial.println("marked: turn a wheel by hand exactly 10 revolutions, then 'e'");
            break;
        case 'd': {
            float d = isnan(a1) ? 0.2f : a1;
            localCommand(zadp::Action::Goto, sn.x + d * cosf(sn.h), sn.y + d * sinf(sn.h), sn.h, 0);
            Serial.printf("drive %.3f m: check with a ruler, the 'e' counts should rise by %.0f on each wheel\n", (double)d,
                          (double)(d / (zadp::kPi * WHEEL_DIAMETER_M) * CPR_LEFT));
            break;
        }
        case 't': {
            float deg = isnan(a1) ? 90.f : a1;
            float h = zadp::wrapAngle(sn.h + deg * zadp::kPi / 180.f);
            localCommand(zadp::Action::Turn, sn.x, sn.y, h, deg >= 0 ? 1 : -1);
            Serial.printf("turn %.0f deg to heading %.1f deg\n", (double)deg, (double)(h * 180.f / zadp::kPi));
            break;
        }
        case 's': {
            CtrlMsg m{};
            m.kind = CTRL_RAW; m.rawSec = 0;                     // ends a raw test (the new stop command does too)
            const bool ok1 = postCtrl(m);
            const bool ok2 = localCommand(zadp::Action::Stop, 0, 0, 0, 0);
            Serial.println(ok1 || ok2 ? "stop" : "command queue full: stop NOT sent, type 's' again");
            break;
        }
        case 'p': {
            if (isnan(a3)) { Serial.println("usage: p x y deg"); break; }
            CtrlMsg m{};
            m.kind = CTRL_POSE; m.x = a1; m.y = a2; m.h = a3 * zadp::kPi / 180.f;
            postCtrl(m);
            Serial.printf("pose set to %.3f %.3f %.1f deg\n", (double)a1, (double)a2, (double)a3);
            break;
        }
        case 'i': printStatus(); break;
        case 'v': statusStream = !statusStream; Serial.printf("status stream %s\n", statusStream ? "on" : "off"); break;
        case 'w': printStackReport(); break;
        default: Serial.println("unknown command, 'h' for help"); break;
    }
}

void consoleTask(void*) {
    static char line[CONSOLE_LINE_MAX + 1];
    size_t len = 0;
    uint32_t lastStatus = 0;
    bool reported = false;
    printHelp();
    for (;;) {
        while (Serial.available()) {
            char ch = (char)Serial.read();
            if (ch == '\n' || ch == '\r') { line[len] = 0; runLine(line); len = 0; }
            else if (len < CONSOLE_LINE_MAX) line[len++] = ch;
        }
        uint32_t ms = millis();
        if (statusStream && ms - lastStatus >= 1000) { lastStatus = ms; printStatus(); }
        if (!reported && ms >= BOOT_STACK_REPORT_MS) { reported = true; printStackReport(); }   // once after boot
        vTaskDelay(pdMS_TO_TICKS(CONSOLE_PERIOD_MS));
    }
}

}  // namespace app
