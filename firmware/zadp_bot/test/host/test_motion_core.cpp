// test_motion_core.cpp : host-side test of include/motion_core.h against a simulated differential-drive robot.
//
// Build (any C++11 or newer compiler), from firmware/zadp_bot:
//   g++ -std=c++17 -O2 -Wall -Wextra -Iinclude test/host/test_motion_core.cpp -o test_motion_core
//   ./test_motion_core            (exit code 0 = all checks passed)
//
// Same checks as test/host/js_mirror/mirror_test.js (which is the version that has been RUN; keep both in step).
// The simulated robot is deliberately NOT the firmware's own model: motor dead band 28 PWM (firmware assumes 20),
// loaded top speed 0.48 m/s (firmware assumes 0.534), motor lag 0.08 s, short-brake lag 0.03 s, encoder
// quantisation (true CPR 2772.1 / 2772.3, floor), wheel slip 1 % (P.bot.slipStd), physics at 1 kHz, camera at
// 30 Hz with 3 mm / 1 deg noise, image age 40 ms (the age field) + 5..15 ms network.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <random>

#include "motion_core.h"

using namespace zadp;

static int g_fail = 0, g_pass = 0;
static void check(bool ok, const char* what) {
    if (ok) ++g_pass; else ++g_fail;
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
}
static const double PI_D = 3.14159265358979323846;

// ------------------------------------------------------------------------------------------- plant
struct Plant {
    Pose pose;
    double wheel[2] = {0, 0}, v[2] = {0, 0}, slip[2] = {1, 1};
    double deadband = 28.0, vmaxLoaded = 0.48, tau = 0.08, tauBrake = 0.03;
    double wheelD[2] = {WHEEL_DIAMETER_M, WHEEL_DIAMETER_M};
    double track = TRACK_M;
    double cpr[2] = {2772.1, 2772.3};
    double slipStd = 0.01, slipCorr = 0.20;
    std::mt19937 rng;
    std::normal_distribution<double> n01{0.0, 1.0};
    std::uniform_real_distribution<double> u01{0.0, 1.0};
    explicit Plant(unsigned seed) : rng(seed) { for (int i = 0; i < 2; ++i) slip[i] = 1 + slipStd * n01(rng); }
    int32_t enc(int i) const { return (int32_t)std::floor(wheel[i] / (2 * PI_D) * cpr[i]); }
    void step(const int pwm[2], double dt) {
        double ds[2];
        for (int i = 0; i < 2; ++i) {
            double u = pwm[i], vss = 0.0, t = tau;
            if (u == 0) t = tauBrake;
            else if (std::fabs(u) > deadband) vss = (u > 0 ? 1 : -1) * (std::fabs(u) - deadband) / (255.0 - deadband) * vmaxLoaded;
            v[i] += (vss - v[i]) * (1 - std::exp(-dt / t));
            double dRot = v[i] * dt;
            wheel[i] += dRot / (WHEEL_DIAMETER_M / 2);
            double a = std::exp(-std::fabs(dRot) / slipCorr);
            slip[i] = 1 + (slip[i] - 1) * a + slipStd * std::sqrt(1 - a * a) * n01(rng);
            ds[i] = dRot * (wheelD[i] / WHEEL_DIAMETER_M) * slip[i];
        }
        pose = arcStep(pose, (float)((ds[0] + ds[1]) / 2), (float)((ds[1] - ds[0]) / track));
        pose.th = wrapAngle(pose.th);
    }
};

// ------------------------------------------------------------------------------------------- harness
struct CamMsg { double deliver, tImage; float age, x, y, h; };

struct Harness {
    Plant plant;
    Truck truck;
    double t = 0.0;
    const double dt;
    const int sub = 5;
    bool camOn = true, keepAlive = true, camNoise = true, ageReported = true;
    float camAge = 0.04f;
    Command last;
    double nextRepeat = 0.0, nextCam = 0.0;
    std::deque<CamMsg> camQ;
    const double clockOffset = 1795023945.0;
    double maxRef = 0.0, maxCmd = 0.0;

    Harness(unsigned seed, Pose start, const Params& P = Params()) : plant(seed), truck(P), dt(P.dt) {
        plant.pose = start;
        truck.begin(start, 0, 0, 0.0);
    }
    bool send(const Command& c) { last = c; nextRepeat = t + 0.1; return truck.onCommand(c, t); }
    void tick() {
        if (keepAlive && last.seq != 0 && t >= nextRepeat) { truck.onCommand(last, t); nextRepeat += 0.1; }
        if (t >= nextCam) {
            nextCam += 1.0 / 30.0;
            if (camOn) {
                double sx = camNoise ? 0.003 : 0.0, sh = camNoise ? PI_D / 180.0 : 0.0;
                CamMsg m;
                m.tImage = t + clockOffset;
                m.age = ageReported ? camAge : -1.f;
                m.deliver = t + camAge + 0.005 + 0.01 * plant.u01(plant.rng);
                m.x = (float)(plant.pose.x + sx * plant.n01(plant.rng));
                m.y = (float)(plant.pose.y + sx * plant.n01(plant.rng));
                m.h = wrapAngle((float)(plant.pose.th + sh * plant.n01(plant.rng)));
                camQ.push_back(m);
            }
        }
        while (!camQ.empty() && camQ.front().deliver <= t) {
            CamMsg m = camQ.front(); camQ.pop_front();
            truck.onCamera(m.x, m.y, m.h, m.tImage, m.age, t);
        }
        truck.step(t, plant.enc(0), plant.enc(1));
        maxRef = std::max(maxRef, (double)std::max(std::fabs(truck.refL()), std::fabs(truck.refR())));
        maxCmd = std::max(maxCmd, (double)std::max(std::fabs(truck.cmdL()), std::fabs(truck.cmdR())));
        int pwm[2] = {(int)std::lround(truck.pwmL()), (int)std::lround(truck.pwmR())};
        for (int k = 0; k < sub; ++k) plant.step(pwm, dt / sub);
        t += dt;
    }
    double runUntilArrived(uint32_t seq, double timeout) {
        double t0 = t;
        while (t - t0 < timeout) { tick(); if (truck.state() == State::Arrived && truck.seq() == seq) return t - t0; }
        return -1.0;
    }
    void run(double seconds) { double t0 = t; while (t - t0 < seconds) tick(); }
    double trueSpeed() const { return std::max(std::fabs(plant.v[0]), std::fabs(plant.v[1])); }
};

static Command gotoCmd(uint32_t seq, float x, float y, float h) {
    Command c; c.seq = seq; c.action = Action::Goto; c.x = x; c.y = y; c.heading = h; c.hasXY = c.hasHeading = true;
    c.speed = 0.2f; return c;
}
static Command turnCmd(uint32_t seq, float h, int dir, float tolDeg = 0.f) {
    Command c; c.seq = seq; c.action = Action::Turn; c.heading = h; c.hasHeading = true; c.dir = dir; c.speed = 0.2f;
    c.tolDeg = tolDeg; return c;
}
static Command simpleCmd(uint32_t seq, Action a) { Command c; c.seq = seq; c.action = a; return c; }
static double dist(const Pose& a, double x, double y) { return std::hypot(a.x - x, a.y - y); }
static double cross(const Pose& p, double x, double y, double h) { return -(p.x - x) * std::sin(h) + (p.y - y) * std::cos(h); }
static double rad2deg(double r) { return r * 180.0 / PI_D; }

// simple uniform generator for the random starts
struct Uni { std::mt19937 g; std::uniform_real_distribution<double> u{0.0, 1.0}; explicit Uni(unsigned s) : g(s) {} double operator()() { return u(g); } };

// ------------------------------------------------------------------------------------------- tests
static void testSemantics() {
    std::printf("\nTEST seq / state semantics (TOPICS.md, fake_bot.py)\n");
    Harness H(601, makePose(0.2f, 0.5f, 0.f)); Truck& T = H.truck;
    check(T.seq() == 0 && T.state() == State::Idle, "before the first command: seq 0, idle");
    bool ack = H.send(gotoCmd(1, 0.5f, 0.5f, 0.f));
    check(ack && T.state() == State::Moving && T.seq() == 1, "goto seq 1: new (ack), moving");
    check(!T.onCommand(gotoCmd(1, 0.5f, 0.5f, 0.f), H.t), "same seq again = repeat (no ack)");
    double t0 = H.t, vAt = -1; bool sawMoving = false;
    while (H.t - t0 < 10) { H.tick(); if (T.state() == State::Moving) sawMoving = true; if (T.state() == State::Arrived) { vAt = std::max(std::fabs(T.speedL()), std::fabs(T.speedR())); break; } }
    std::printf("  goto arrived after %.2f s, measured wheel speed at arrival %.4f m/s\n", H.t - t0, vAt);
    check(sawMoving && vAt >= 0 && vAt < STOPPED_SPEED && H.trueSpeed() < 0.003, "arrived only with the wheels stopped");
    H.run(0.5);
    check(T.state() == State::Arrived && T.seq() == 1, "repeats of a finished command keep 'arrived' (no restart)");
    H.send(gotoCmd(2, 0.6f, 0.55f, 0.4f)); bool sawTurning = false; t0 = H.t;
    while (H.t - t0 < 10) { H.tick(); if (T.state() == State::Turning) sawTurning = true; if (T.state() == State::Arrived) break; }
    check(!sawTurning && T.state() == State::Arrived, "a goto never turns in place at its end");
    H.send(turnCmd(3, -1.5f, 1)); check(T.state() == State::Turning, "turn: state turning");
    check(H.runUntilArrived(3, 10) > 0, "turn arrives");
    H.send(turnCmd(31, 0.3f, -1, 1.f)); double t1 = H.runUntilArrived(31, 10);
    double e1 = std::fabs(rad2deg(wrapAngle(T.est().th - 0.3f)));
    std::printf("  turn with tol 1 deg: heading error %.2f deg (estimate)\n", e1);
    check(t1 > 0 && e1 < 1.0, "a turn with `tol` 1 deg stops within 1 deg");
    H.keepAlive = false; H.send(simpleCmd(4, Action::Dump)); check(T.state() == State::Dumping, "dump: state dumping");
    t0 = H.t; double tArr = -1; bool still = true;
    while (H.t - t0 < 6) { H.tick(); if (T.pwmL() != 0 || T.pwmR() != 0) still = false; if (T.state() == State::Arrived) { tArr = H.t - t0; break; } }
    std::printf("  dump arrived after %.3f s (dumpTime 4.0)\n", tArr);
    check(std::fabs(tArr - 4) < 0.011 && still, "dump: wheels still for 4 s, then arrived");
    H.keepAlive = true; H.send(gotoCmd(5, 0.2f, 0.5f, (float)PI_D)); H.run(1.5);
    check(T.state() == State::Moving, "goto seq 5 under way");
    ack = H.send(simpleCmd(6, Action::Stop)); check(ack && T.state() == State::Idle && T.seq() == 6, "stop: new (ack), idle at once");
    double tStop = H.runUntilArrived(6, 2);
    check(tStop > 0 && H.trueSpeed() < 0.003, "stop: 'arrived' with the stop's seq once the wheels stopped");
    H.send(gotoCmd(7, 0.8f, 0.5f, 0.f)); H.run(1.0);
    H.send(gotoCmd(8, 0.3f, 0.3f, std::atan2(0.3f - T.est().y, 0.3f - T.est().x)));
    double tt = H.runUntilArrived(8, 20);
    check(tt > 0 && dist(T.est(), 0.3, 0.3) < 0.0101, "a different seq replaces the active command");
    ack = T.onCommand(simpleCmd(3, Action::Dump), H.t); check(ack && T.state() == State::Dumping && T.seq() == 3, "seq is not an order: a lower seq is a new command");
    ack = H.send(simpleCmd(9, Action::Unknown)); check(ack && T.state() == State::Idle && T.seq() == 9, "unknown action: accepted seq, idle");
    Command l0 = gotoCmd(0, 0.4f, 0.4f, 0.f); bool a1 = T.onCommand(l0, H.t), a2 = T.onCommand(l0, H.t);
    check(a1 && a2 && T.seq() == 0, "seq 0 (first-draft message): every message counts as new");
    Command nh = gotoCmd(10, T.est().x + 0.1f, T.est().y, 0.f); nh.hasHeading = false; H.send(nh);
    check(H.runUntilArrived(10, 10) > 0, "goto without heading arrives (line from here to the target)");
    Command big = gotoCmd(4000000000u, T.est().x + 0.05f, T.est().y, 0.f);
    check(H.send(big) && T.seq() == 4000000000u, "seq above 2^31 accepted (uint32)");
    Harness W(602, makePose(0.2f, 0.5f, 0.f)); W.camOn = false; W.send(gotoCmd(1, 0.8f, 0.5f, 0.f)); W.run(0.5); W.keepAlive = false; W.run(0.6);
    bool wasIdle = W.truck.state() == State::Idle && W.truck.stalled(); W.keepAlive = true; W.send(simpleCmd(2, Action::Dump)); W.tick();
    check(wasIdle && W.truck.state() == State::Dumping && !W.truck.stalled(), "after a watchdog stop a new dump reports dumping");
}

static void testGoto(bool cam) {
    const int n = 40; std::printf("\nTEST goto from random starts (%d runs, camera %s)\n", n, cam ? "on" : "off");
    Uni r(cam ? 11 : 12);
    int ok = 0; double wE = 0, wT = 0;
    for (int k = 0; k < n; ++k) {
        Pose s = makePose((float)(0.1 + 0.8 * r()), (float)(0.1 + 0.8 * r()), (float)(-PI_D + 2 * PI_D * r()));
        double tx, ty; do { tx = 0.1 + 0.8 * r(); ty = 0.1 + 0.8 * r(); } while (std::hypot(tx - s.x, ty - s.y) < 0.05);
        float h = (float)std::atan2(ty - s.y, tx - s.x);
        Harness H(100 + k, s); H.camOn = cam; H.run(0.2); H.send(gotoCmd(1, (float)tx, (float)ty, h));
        double tt = H.runUntilArrived(1, 30);
        double eE = dist(H.truck.est(), tx, ty), eT = dist(H.plant.pose, tx, ty);
        ok += tt > 0 && eE <= 0.0101 && H.trueSpeed() < 0.005;
        wE = std::max(wE, eE); wT = std::max(wT, eT);
    }
    std::printf("  arrived %d/%d, worst |est-target| %.1f mm, worst |true-target| %.1f mm\n", ok, n, wE * 1000, wT * 1000);
    check(ok == n, "every goto arrives (estimate within 10 mm, wheels stopped)");
    if (cam) check(wT < 0.010, "with the camera the TRUE position is within 10 mm of the target");
}

static double lineRuns(float y0, int n) {
    double wAll = 0;
    for (int k = 0; k < n; ++k) {
        Harness H(700 + k, makePose(0.2f, y0, 0.f)); H.run(0.2); H.send(gotoCmd(1, 0.8f, 0.5f, 0.f));
        double t0 = H.t;
        while (H.t - t0 < 10) { H.tick(); wAll = std::max(wAll, std::fabs(cross(H.plant.pose, 0.8, 0.5, 0))); if (H.truck.state() == State::Arrived) break; }
    }
    return wAll;
}
static void testLine() {
    std::printf("\nTEST line following (0.6 m leg, start on the line, 1 %% slip, camera on, 20 runs)\n");
    double w = lineRuns(0.5f, 20);
    std::printf("  worst TRUE cross-track during the drive %.1f mm\n", w * 1000);
    // parity with the twin (same leg, twin model, 20 runs, 2026-11-19: worst 8.40 mm), see mirror_test.js
    check(w <= 0.0084, "cross-track no worse than the twin in the same scenario (twin worst 8.4 mm)");
}

static void testTurn(bool cam) {
    const int n = 60; std::printf("\nTEST turn in place (%d runs, dir -1/0/+1, camera %s)\n", n, cam ? "on" : "off");
    Uni r(cam ? 22 : 21);
    int ok = 0, dirOk = 0;
    for (int k = 0; k < n; ++k) {
        Pose s = makePose(0.5f, 0.5f, (float)(-PI_D + 2 * PI_D * r())); float h = (float)(-PI_D + 2 * PI_D * r()); int dir = (k % 3) - 1;
        Harness H(200 + k, s); H.camOn = cam; H.run(0.2);
        float th0 = H.truck.est().th; H.send(turnCmd(1, h, dir));
        double swept = 0; float prev = th0; bool arrived = false; double t0 = H.t;
        while (H.t - t0 < 10) { H.tick(); float th = H.truck.est().th; swept += wrapAngle(th - prev); prev = th;
            if (H.truck.state() == State::Arrived && H.truck.seq() == 1) { arrived = true; break; } }
        double e = std::fabs(rad2deg(wrapAngle(H.truck.est().th - h))), eT = std::fabs(rad2deg(wrapAngle(H.plant.pose.th - h)));
        ok += arrived && (cam ? eT <= 4.5 : e <= 3);
        double need = dir == 0 ? 0 : pyMod(dir * (h - th0), kTwoPi);
        dirOk += dir == 0 || need < 3 * PI_D / 180 || need > 2 * PI_D - TURN_WRAP_SLACK || swept * dir > 0;
    }
    std::printf("  arrived within tolerance %d/%d, direction obeyed %d/%d\n", ok, n, dirOk, n);
    check(ok == n, cam ? "every turn ends with the TRUE heading within 4.5 deg" : "every turn ends within 3 deg (estimate)");
    check(dirOk == n, "a forced direction (dir = +1 / -1) is obeyed");
}

static void testSpeedLimit() {
    std::printf("\nTEST wheel speed limit\n");
    Harness H(301, makePose(0.1f, 0.1f, 0.f)); Command c = gotoCmd(1, 0.9f, 0.9f, (float)(PI_D / 4)); c.speed = 5.f;
    H.send(c); H.runUntilArrived(1, 30); H.send(turnCmd(2, 3.0f, -1)); H.runUntilArrived(2, 10);
    Params P; double wl = 0;
    for (int i = 0; i < 2000; ++i) {
        float e = (float)(-PI_D + 2 * PI_D * i / 2000); bool arr, rot = false, rot2 = false;
        WheelCmd a = gotoLaw(makePose(0, 0, e), 1, 0, P.cruise, P, arr, rot), a2 = gotoLaw(makePose(0, 0, e), 0.02f, 0, P.cruise, P, arr, rot2);
        WheelCmd b = turnLaw(0, e, 0, P.headTol, P, arr);
        wl = std::max({wl, (double)std::fabs(a.vL), (double)std::fabs(a.vR), (double)std::fabs(a2.vL), (double)std::fabs(a2.vR), (double)std::fabs(b.vL), (double)std::fabs(b.vR)});
    }
    std::printf("  max wheel speed command %.3f m/s, max from the laws %.3f m/s\n", H.maxCmd, wl);
    check(H.maxRef <= MAX_WHEEL_SPEED && H.maxCmd <= MAX_WHEEL_SPEED && wl <= MAX_WHEEL_SPEED, "no wheel speed command exceeds 0.534 m/s");
}

static void testWatchdog() {
    std::printf("\nTEST watchdog (silence 300 ms)\n");
    Harness H(401, makePose(0.1f, 0.5f, 0.f)); H.camOn = false;
    H.send(gotoCmd(1, 0.9f, 0.5f, 0.f)); H.run(1.0);
    H.keepAlive = false; double ts = H.t, tI = -1, tS = -1;
    while (H.t - ts < 2) { H.tick(); if (tI < 0 && H.truck.state() == State::Idle) tI = H.t - ts; if (tI >= 0 && tS < 0 && H.trueSpeed() < 0.002) tS = H.t - ts; }
    Pose p1 = H.plant.pose; H.run(1.0); double creep = dist(H.plant.pose, p1.x, p1.y);
    std::printf("  idle %.3f s after silence, wheels stopped %.3f s after, creep %.2f mm\n", tI, tS, creep * 1000);
    check(tI > 0 && tI <= 0.31, "state idle within 300 ms of silence");
    check(tS > 0 && tS <= 0.31 + 0.2 / MAX_ACCEL + 0.1, "wheels stopped within 300 ms + ramp-down");
    check(creep < 0.0005, "robot stays put while silent");
    H.keepAlive = true; H.nextRepeat = H.t; H.tick(); bool resumed = H.truck.state() == State::Moving;
    check(resumed && H.runUntilArrived(1, 20) > 0, "keeps the goal and resumes when repeats return");
    Harness G(402, makePose(0.1f, 0.5f, 0.f)); G.keepAlive = false; G.send(gotoCmd(1, 0.9f, 0.5f, 0.f)); G.run(1.5);
    check(dist(G.plant.pose, 0.1, 0.5) < 0.05 && G.truck.state() == State::Idle, "a command that is never repeated stops after 300 ms");
}

static double latencyRun(bool ageReported) {
    Params P0; if (!ageReported) P0.camDefaultLag = 0.f;
    Harness L(510, makePose(0.1f, 0.5f, 0.f), P0); L.camNoise = false; L.ageReported = ageReported;
    L.run(0.3); L.send(gotoCmd(1, 0.9f, 0.5f, 0.f)); double s = 0; int n = 0; double t0 = L.t;
    while (L.t - t0 < 4) { L.tick(); if (L.truck.state() == State::Moving && L.trueSpeed() > 0.15) { s += std::fabs(L.plant.pose.x - L.truck.est().x); ++n; } }
    return s / std::max(n, 1);
}

static void testCamera() {
    std::printf("\nTEST camera fusion\n");
    double eComp = latencyRun(true), eLag = latencyRun(false);
    std::printf("  at cruise, mean along-track estimate error: with age %.2f mm, uncompensated %.2f mm\n", eComp * 1000, eLag * 1000);
    check(eComp < 0.0015 && eLag > 2 * eComp, "latency compensated: estimate does not lag the truth");
    Truck T; T.begin(makePose(0.5f, 0.5f, 0.f), 0, 0, 1.0); T.setPose(makePose(0.5f, 0.5f, 0.f));
    double now = 1.0; const double tImg = 1000.0; for (int i = 0; i < 20; ++i) { now += 0.005; T.step(now, 0, 0); }
    T.onCamera(0.53f, 0.5f, 0.f, tImg, 0.04f, now); double e1 = 0.53 - T.est().x;
    T.onCamera(0.53f, 0.5f, 0.f, tImg + 0.033, 0.04f, now + 0.033); double e2 = 0.53 - T.est().x;
    check(std::fabs(e1 - 0.021) < 1e-4 && std::fabs(e2 - 0.0147) < 1e-4, "complementary gain 0.3 per fix");
    T.setPose(makePose(0.5f, 0.5f, deg2radf(179))); T.onCamera(0.5f, 0.5f, deg2radf(-179), tImg + 0.066, 0.04f, now + 0.066);
    check(std::fabs(std::fabs(rad2deg(T.est().th)) - 180) < 0.7, "heading wrapped through +-180 (not via 0)");
    T.setPose(makePose(0.5f, 0.5f, 0.f)); CamResult r = T.onCamera(0.7f, 0.5f, 0.f, tImg + 0.1, 0.04f, now + 0.1);
    check(r == CamResult::RejectedJump && std::fabs(T.est().x - 0.5) < 1e-6, "a 200 mm jump is rejected");
    int k = 1; double tt = 0.1;
    while (r != CamResult::Relocked && k < 50) { tt += 0.033; r = T.onCamera(0.7f, 0.5f, 0.f, tImg + tt, 0.04f, now + tt); ++k; }
    check(r == CamResult::Relocked && k == CAM_RELOCK_COUNT && std::fabs(T.est().x - 0.7) < 1e-6, "after 10 consistent fixes the camera wins");
    tt += 0.033; r = T.onCamera(0.7f, 0.5f, 0.f, tImg + tt - 0.5, 0.04f, now + tt); check(r == CamResult::RejectedOrder, "an out-of-order fix is rejected");
    r = T.onCamera(0.7f, 0.5f, 0.f, tImg + tt + 0.01, 0.30f, now + tt + 0.40); check(r == CamResult::RejectedOld, "a fix with age 0.30 s is rejected");
    r = T.onCamera(0.7f, 0.5f, 0.f, tImg + tt + 0.05, 0.04f, now + tt + 0.05); check(r == CamResult::Accepted, "the next timely fix is accepted again");
}

// same checks as mirror_test.js testCamStale
static void testCamStale() {
    std::printf("\nTEST camera watchdog (no accepted fix for 1 s while driving or turning)\n");
    Harness H(801, makePose(0.1f, 0.5f, 0.f)); Truck& T = H.truck;
    check(T.camAgeMs(H.t) == -1, "before the first fix the telemetry field cam is -1");
    H.run(0.3); H.send(gotoCmd(1, 0.9f, 0.5f, 0.f)); H.run(1.0);
    int32_t ageOn = T.camAgeMs(H.t);
    check(T.state() == State::Moving && !T.camStale() && ageOn >= 0 && ageOn < 100, "camera on: moving, cam age below 100 ms");
    H.camOn = false; double ts = H.t, tI = -1, tS = -1;
    while (H.t - ts < 3) { H.tick(); if (tI < 0 && T.state() == State::Idle) tI = H.t - ts; if (tI >= 0 && tS < 0 && H.trueSpeed() < 0.002) tS = H.t - ts; }
    Pose p1 = H.plant.pose; H.run(1.0); double creep = dist(H.plant.pose, p1.x, p1.y);
    std::printf("  camera off: idle %.3f s later, wheels stopped %.3f s later, creep %.2f mm, cam age %ld ms\n", tI, tS, creep * 1000, (long)T.camAgeMs(H.t));
    check(tI > CAM_STALE_S - 0.01 && tI <= CAM_STALE_S + 0.065, "state idle 1 s after the last accepted fix (not earlier)");
    check(tS > 0 && tS <= CAM_STALE_S + 0.065 + 0.2 / MAX_ACCEL + 0.1, "wheels stopped within 1 s + ramp-down");
    check(creep < 0.0005 && T.stalled() && T.camStale() && T.seq() == 1, "robot stays put, goal kept (same seq), although cmd repeats keep coming");
    check(T.camAgeMs(H.t) > 1000, "cam age in telemetry keeps counting above 1000 ms");
    bool ack = H.send(gotoCmd(2, 0.9f, 0.5f, 0.f)); H.run(0.5);
    check(ack && T.state() == State::Idle && T.camStale() && H.trueSpeed() < 0.002, "a NEW command does not resume: still no camera, still stopped");
    H.camOn = true; H.nextCam = H.t; double tr = H.t, tM = -1;
    while (H.t - tr < 1 && tM < 0) { H.tick(); if (T.state() == State::Moving) tM = H.t - tr; }
    double tt = H.runUntilArrived(2, 20);
    std::printf("  camera back: moving %.3f s later, arrived after %.2f s\n", tM, tt);
    check(tM > 0 && tM <= 0.07 && !T.camStale(), "resumes with the first accepted fix");
    check(tt > 0 && dist(H.plant.pose, 0.9, 0.5) < 0.010, "then arrives (true position within 10 mm)");
    Params Pv; Pv.cruise = 0.03f;                                // a turn slowed down so that it needs more than 1 s
    Harness V(803, makePose(0.5f, 0.5f, 0.f), Pv); V.run(0.3); V.send(turnCmd(1, 3.0f, 1)); V.run(0.3); V.camOn = false;
    double v0 = V.t, vI = -1; while (V.t - v0 < 3 && vI < 0) { V.tick(); if (V.truck.state() == State::Idle) vI = V.t - v0; }
    check(vI > CAM_STALE_S - 0.01 && vI <= CAM_STALE_S + 0.065 && V.truck.camStale(), "a turn is stopped the same way");
    Harness G(804, makePose(0.1f, 0.5f, 0.f)); G.camOn = false; G.send(gotoCmd(1, 0.9f, 0.5f, 0.f));
    double tg = G.runUntilArrived(1, 20);
    check(tg > 2 && !G.truck.camStale() && G.truck.camAgeMs(G.t) == -1, "a robot that never saw a camera drives longer than 1 s and arrives (cam -1)");
    Harness B(805, makePose(0.1f, 0.5f, 0.f)); B.camOn = false; B.truck.setPose(makePose(0.1f, 0.5f, 0.f)); B.send(gotoCmd(1, 0.9f, 0.5f, 0.f));
    check(B.runUntilArrived(1, 20) > 2 && B.truck.camAgeMs(B.t) == -1, "a bench pose (setPose) does not arm the camera watchdog");
    Harness L(806, makePose(0.1f, 0.5f, 0.f)); L.run(0.3); L.camOn = false; L.keepAlive = false;
    Command lc = gotoCmd(1, 0.9f, 0.5f, 0.f); lc.local = true; L.send(lc);
    check(L.runUntilArrived(1, 20) > 2 && !L.truck.camStale(), "a serial bench command (local) is not stopped by the camera watchdog");
    Harness D(807, makePose(0.5f, 0.5f, 0.f)); D.run(0.3); D.camOn = false; D.send(simpleCmd(1, Action::Dump));
    check(std::fabs(D.runUntilArrived(1, 6) - 4) < 0.011, "a dump keeps timing without the camera (wheels are still anyway)");
}

// same checks as mirror_test.js testCamOrder
static void testCamOrder() {
    std::printf("\nTEST camera order check is bounded (camera clock stepped back)\n");
    Truck T; T.begin(makePose(0.5f, 0.5f, 0.f), 0, 0, 1.0);
    double now = 1.0; for (int i = 0; i < 20; ++i) { now += 0.005; T.step(now, 0, 0); }
    auto fix = [&](double tImg) { now += 0.033; T.step(now, 0, 0); return T.onCamera(0.5f, 0.5f, 0.f, tImg, 0.04f, now); };
    CamResult r = fix(2000.0); check(r == CamResult::Initialised, "first fix initialises");
    r = fix(2000.033); check(r == CamResult::Accepted, "next fix in order accepted");
    bool lone = true;
    for (int i = 0; i < 30; ++i) { if (fix(1999.0) != CamResult::RejectedOrder) lone = false; if (fix(2000.1 + 0.033 * i) != CamResult::Accepted) lone = false; }
    check(lone, "single out-of-order fixes between good ones are always rejected (the count resets)");
    int k = 0; double tb = 1500.0; r = CamResult::RejectedOrder;
    while (r == CamResult::RejectedOrder && k < 50) { tb += 0.033; r = fix(tb); ++k; }
    std::printf("  clock 500 s back: accepted again at fix %d (%d rejected)\n", k, k - 1);
    check(r == CamResult::Accepted && k == CAM_RELOCK_COUNT, "after 10 fixes in a row from the stepped-back clock the order check re-bases and accepts");
    tb += 0.033; r = fix(tb); check(r == CamResult::Accepted, "the following fix on the new time base is accepted");
    r = fix(tb - 0.1); check(r == CamResult::RejectedOrder, "an out-of-order fix on the new time base is rejected again");
    check(T.camAgeMs(now) >= 0 && T.camAgeMs(now) < 100, "cam age follows the accepted fixes");
}

int main() {
    std::printf("motion_core host test: CONTROL_HZ %d, cruise %.2f m/s, fusion gain %.2f\n", CONTROL_HZ, (double)CRUISE_SPEED, (double)CAM_FUSION_GAIN);
    testSemantics(); testGoto(true); testGoto(false); testLine(); testTurn(false); testTurn(true); testSpeedLimit(); testWatchdog(); testCamera(); testCamStale(); testCamOrder();
    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
