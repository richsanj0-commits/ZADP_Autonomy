// motion_core.h : the maths of one ZADP truck robot. PORTABLE C++11 (no Arduino headers), so the host test
// (test/host) runs exactly this code on a PC.
//
// Mirrors the twin: twin3d/bot/bot_odometry.m, bot_controller.m (capture radius, rotate hysteresis),
// pose_fusion.m (latency-compensated fusion), brain/brain_step.m (drive along the leg line with a carrot,
// coast-predicted braking, turnCmd), and twin3d/link/py/fake_bot.py (command / seq / state semantics).
// Contract: twin3d/link/TOPICS.md.
//
// One control tick (Truck::step, CONTROL_HZ):
//   encoder counts -> odometry (exact arc, chord form) -> wheel speed estimate
//   -> command state machine + go-to-point / turn law -> wheel speed commands
//   -> wheel speed limit -> acceleration limit -> per-wheel PI(D) with feed-forward -> signed PWM
// Camera fixes (Truck::onCamera) are moved forward by the odometry since the image time, then pull the
// estimate toward them.
#pragma once

#include <cmath>
#include <cstdint>

#include "config.h"

namespace zadp {

static const float kPi = 3.14159265358979f;
static const float kTwoPi = 6.28318530717959f;

inline float wrapAngle(float a) { return std::atan2(std::sin(a), std::cos(a)); }
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float sgnf(float v) { return v > 0.f ? 1.f : (v < 0.f ? -1.f : 0.f); }
inline float deg2radf(float d) { return d * kPi / 180.f; }
inline float maxf(float a, float b) { return a > b ? a : b; }
inline float minf(float a, float b) { return a < b ? a : b; }
// Python's a % m (result in [0, m)), as used by fake_bot.py / MATLAB mod in turnCmd
inline float pyMod(float a, float m) { return a - m * std::floor(a / m); }

// ------------------------------------------------------------------------------------------- parameters
// Defaults come from config.h. The host test changes some of them (never the controller numbers).
struct Params {
    // geometry
    float wheelD = WHEEL_DIAMETER_M;
    float cprL = CPR_LEFT, cprR = CPR_RIGHT;
    float track = TRACK_M * TRACK_SCALE;                         // rule: effective track = P.bot.trackEff
    // limits
    float maxWheel = MAX_WHEEL_SPEED, cruise = CRUISE_SPEED, minGoto = MIN_GOTO_SPEED, accel = MAX_ACCEL;
    float motorTau = MOTOR_TAU_S;
    // laws
    float kpHead = KP_HEADING, kpDist = KP_DISTANCE;
    float rotThresh = deg2radf(ROTATE_THRESH_DEG), rotExit = deg2radf(ROTATE_EXIT_DEG), decelFrac = DECEL_FRAC;
    float arriveTol = ARRIVE_TOL_M, headTol = deg2radf(HEADING_TOL_DEG), minTurnRate = MIN_TURN_RATE;
    float capture = CAPTURE_RADIUS_M, slide = SLIDE_DIST_M, lookahead = LOOKAHEAD_M, turnSlack = TURN_WRAP_SLACK;
    float stoppedSpeed = STOPPED_SPEED, dumpTime = DUMP_TIME_S, watchdog = WATCHDOG_S;
    // camera
    float camGain = CAM_FUSION_GAIN, camNetDelay = CAM_NET_DELAY_S, camDefaultLag = CAM_DEFAULT_LAG_S;
    float camMaxAge = CAM_MAX_AGE_S;
    float camMaxJump = CAM_MAX_JUMP_M, camMaxJumpRad = deg2radf(CAM_MAX_JUMP_DEG);
    int camRelock = CAM_RELOCK_COUNT;
    // wheel loop
    float dt = 1.0f / CONTROL_HZ;
    int velWindow = VEL_WINDOW;
    float pwmMax = PWM_MAX, deadband = PWM_DEADBAND, pwmPerMps = PWM_PER_MPS;
    float kp = WHEEL_KP, ki = WHEEL_KI, kd = WHEEL_KD, iLimit = WHEEL_I_LIMIT, refZero = REF_ZERO;

    float mPerCountL() const { return kPi * wheelD / cprL; }
    float mPerCountR() const { return kPi * wheelD / cprR; }
};

// ------------------------------------------------------------------------------------------- odometry
struct Pose { float x = 0.f, y = 0.f, th = 0.f; };
inline Pose makePose(float x, float y, float th) { Pose p; p.x = x; p.y = y; p.th = th; return p; }

// exact arc (bot_odometry.m, fake_bot.py arc), written in the chord form: identical maths to
// r*(sin(th+dth)-sin(th)), but stable in float32 for every dth. rule: no r = ds/dth difference of sines, which
// loses the whole step when th + dth rounds to th (CPR_LEFT != CPR_RIGHT gives dth ~ 4e-8 rad at equal counts).
inline Pose arcStep(const Pose& p, float ds, float dth) {
    Pose q = p;
    const float h = 0.5f * dth;
    const float k = std::fabs(h) < 1e-3f ? 1.f - h * h / 6.f : std::sin(h) / h;   // sin(h)/h, series near 0
    q.x = p.x + ds * k * std::cos(p.th + h);
    q.y = p.y + ds * k * std::sin(p.th + h);
    q.th = p.th + dth;
    return q;
}

// dead reckoning from encoder count deltas (bot_odometry.m; per-wheel CPR, heading kept wrapped)
inline Pose odometry(const Pose& p, int32_t dL, int32_t dR, const Params& P) {
    float sl = dL * P.mPerCountL(), sr = dR * P.mPerCountR();
    Pose q = arcStep(p, 0.5f * (sl + sr), (sr - sl) / P.track);
    q.th = wrapAngle(q.th);
    return q;
}

// ------------------------------------------------------------------------------------------- wheel speed
// speed = count change over the last `window` ticks (fewer right after start)
struct SpeedEstimator {
    enum { N = 32 };
    int32_t hist[N];
    int head = 0, filled = 0;
    SpeedEstimator() { for (int i = 0; i < N; ++i) hist[i] = 0; }
    float update(int32_t count, float mPerCount, int window, float dt) {
        hist[head] = count;
        int w = window < filled ? window : filled;
        if (w > N - 1) w = N - 1;
        float v = 0.f;
        if (w > 0) v = (float)(count - hist[(head - w + N) % N]) * mPerCount / (w * dt);
        head = (head + 1) % N;
        if (filled < N - 1) ++filled;
        return v;
    }
};

// per-wheel PI(D) with feed-forward; returns signed PWM in [-pwmMax, pwmMax]
struct WheelPI {
    float integ = 0.f, prevErr = 0.f;
    void reset() { integ = 0.f; prevErr = 0.f; }
    static float feedForward(float v, const Params& P) {
        if (std::fabs(v) < P.refZero) return 0.f;
        return sgnf(v) * (P.deadband + std::fabs(v) * P.pwmPerMps);   // rule: FF = deadband + slope * |v|
    }
    float update(float ref, float meas, float dt, const Params& P) {
        // rule: zero reference = no drive at all (the accel-limited reference only reaches zero after the ramp)
        if (std::fabs(ref) < P.refZero) { reset(); return 0.f; }
        float e = ref - meas;
        float d = (e - prevErr) / dt;
        prevErr = e;
        float uNoI = feedForward(ref, P) + P.kp * e + P.kd * d;
        float iNew = clampf(integ + P.ki * e * dt, -P.iLimit, P.iLimit);   // rule: integral clamped
        float u = uNoI + iNew;
        // rule (anti-windup): no integration while saturated in the direction of the error
        if ((u > P.pwmMax && e > 0.f) || (u < -P.pwmMax && e < 0.f)) u = uNoI + integ;
        else integ = iNew;
        return clampf(u, -P.pwmMax, P.pwmMax);
    }
};

// ------------------------------------------------------------------------------------------- laws
struct WheelCmd { float vL = 0.f, vR = 0.f; };
inline WheelCmd makeCmd(float v, float w, float b) { WheelCmd c; c.vL = v - w * b / 2; c.vR = v + w * b / 2; return c; }

// go-to-point (bot_controller.m = fake_bot.goto_cmd). cruise = the command's speed cap.
// rotating (in/out) = rotate-in-place mode (hysteresis state).
inline WheelCmd gotoLaw(const Pose& pe, float tx, float ty, float cruise, const Params& P, bool& arrived, bool& rotating) {
    float dx = tx - pe.x, dy = ty - pe.y;
    float d = std::sqrt(dx * dx + dy * dy);
    arrived = d < P.arriveTol;                                   // rule: arrive within arriveTol (5 mm)
    if (arrived) { rotating = false; return WheelCmd(); }
    float e = wrapAngle(std::atan2(dy, dx) - pe.th);
    // rule: inside the capture radius (3 cm) NEVER rotate in place; outside rotate above 25 deg and keep
    // rotating until below 15 deg (hysteresis)
    if (d < P.capture) rotating = false;
    else if (rotating) rotating = std::fabs(e) > P.rotExit;
    else rotating = std::fabs(e) > P.rotThresh;
    float v, w;
    if (rotating) {
        float wmax = cruise / (P.track / 2);
        w = clampf(P.kpHead * e, -wmax, wmax);
        v = 0.f;
    } else {                                                     // rule: v = min(cruise, 2.5 d, sqrt(2*0.25*d)) cos e
        v = minf(minf(cruise, P.kpDist * d), std::sqrt(2.f * P.decelFrac * P.accel * d)) * std::cos(e);
        float eS = e;
        if (std::cos(e) < 0.f) eS = wrapAngle(e + kPi);          // rule: backing up steers the tail to the target
        w = P.kpHead * eS * minf(1.f, d / P.capture);            // rule: yaw faded out inside the capture radius
    }
    return makeCmd(v, w, P.track);
}

// in-place turn (brain_step.m turnCmd = fake_bot.turn_cmd). dir +1 CCW, -1 CW, 0 shortest. tol in rad.
inline WheelCmd turnLaw(float th, float h, int dir, float tol, const Params& P, bool& arrived) {
    if (dir == 0) dir = wrapAngle(h - th) >= 0.f ? 1 : -1;
    float e = pyMod(dir * (h - th) + P.turnSlack, kTwoPi) - P.turnSlack;  // remaining angle in direction dir
    arrived = std::fabs(e) < tol;                                // rule: stop within the command's tol
    if (arrived) return WheelCmd();
    float wmax = P.cruise / (P.track / 2);
    // rule: yaw rate min(5.3, max(0.3, 3|e|)) rad/s
    float w = dir * sgnf(e) * minf(wmax, maxf(P.minTurnRate, P.kpHead * std::fabs(e)));
    return makeCmd(0.f, w, P.track);
}

// distance the robot still rolls after the wheel command goes to zero (brain_step.m): first-order motor lag
// coasts v*tau while v/tau <= accel, above that the accel limit brakes first: v^2/(2a) + a*tau^2/2
inline float coastDistance(float v, const Params& P) {
    v = maxf(v, 0.f);
    float a = P.accel, tau = P.motorTau;
    return v <= a * tau ? v * tau : v * v / (2.f * a) + a * tau * tau / 2.f;
}

// ------------------------------------------------------------------------------------------- commands
enum class Action : uint8_t { Unknown, Goto, Turn, Dump, Stop };
enum class State : uint8_t { Idle, Moving, Turning, Dumping, Arrived };
enum class CamResult : uint8_t { Accepted, Initialised, Relocked, RejectedJump, RejectedOld, RejectedOrder };

inline const char* stateName(State s) {
    switch (s) {
        case State::Moving: return "moving";
        case State::Turning: return "turning";
        case State::Dumping: return "dumping";
        case State::Arrived: return "arrived";
        default: return "idle";
    }
}

struct Command {
    uint32_t seq = 0;       // contract: opaque 32-bit unsigned id
    Action action = Action::Unknown;
    float x = 0.f, y = 0.f, heading = 0.f;
    bool hasXY = false, hasHeading = false;
    float speed = 0.f;      // <= 0: default (CRUISE_SPEED)
    int dir = 0;            // turn only
    float tolDeg = 0.f;     // turn only; <= 0: default HEADING_TOL_DEG
    bool local = false;     // bench command from the serial port: exempt from the watchdog
};

class Truck {
public:
    explicit Truck(const Params& p = Params()) : P(p) {}

    void begin(const Pose& start, int32_t encL, int32_t encR, double now = 0.0) {
        est_ = start; odom_ = start;
        lastEncL_ = encL; lastEncR_ = encR;
        for (int i = 0; i < H; ++i) { histT_[i] = now; hist_[i] = start; }
        histHead_ = 0; histFilled_ = 1;
        state_ = State::Idle; seq_ = 0; kind_ = Kind::None; stalled_ = false; rot_ = false;
        refL_ = refR_ = 0.f; pwmL_ = pwmR_ = 0.f;
        piL_.reset(); piR_.reset();
    }

    // A cmd message (new or repeat). Returns true when it is a NEW command: publish telemetry at once (ack).
    bool onCommand(const Command& c, double now) {
        lastRx_ = now;                                           // rule: every cmd message feeds the watchdog
        if (c.seq == seq_ && c.seq != 0) return false;           // rule: same seq = repeat (keep-alive); seq 0 = new
        seq_ = c.seq;
        raw_ = false;                                            // rule: a new command (broker or bench) ends a raw test
        stalled_ = false;                                        // rule: a new command clears a watchdog stop
        local_ = c.local;
        rot_ = false;
        float speed = c.speed > 0.f ? c.speed : P.cruise;        // rule: speed default 0.20, capped at cruise
        speed = minf(speed, P.cruise);
        Action a = c.action;
        if ((a == Action::Goto && !c.hasXY) || (a == Action::Turn && !c.hasHeading)) a = Action::Unknown;
        switch (a) {
            case Action::Goto:
                kind_ = Kind::Goto; phase_ = Phase::Drive;
                tx_ = c.x; ty_ = c.y;
                // rule: no heading = the line from here to the target
                h_ = c.hasHeading ? c.heading : std::atan2(c.y - est_.y, c.x - est_.x);
                speed_ = maxf(speed, P.minGoto);
                state_ = State::Moving;
                break;
            case Action::Turn:
                kind_ = Kind::Turn; phase_ = Phase::Turn;
                h_ = c.heading; dir_ = c.dir > 0 ? 1 : (c.dir < 0 ? -1 : 0);
                tol_ = c.tolDeg > 0.f ? deg2radf(c.tolDeg) : P.headTol;   // rule: tol (deg) from the command
                state_ = State::Turning;
                break;
            case Action::Dump:
                kind_ = Kind::Dump; phase_ = Phase::Dump; timer_ = 0.f;
                state_ = State::Dumping;
                break;
            case Action::Stop:
                kind_ = Kind::Stop; phase_ = Phase::Stop;
                state_ = State::Idle;                            // rule: idle while ramping down, arrived when stopped
                break;
            default:                                             // rule: unknown action = cancel, wheels to zero, idle
                kind_ = Kind::None; phase_ = Phase::None;
                state_ = State::Idle;
                break;
        }
        return true;
    }

    // Overhead camera fix of THIS robot. tImage = the image's Unix time on the camera PC (<= 0: unknown, no
    // order check), age = seconds from the image to its publication on the camera PC (< 0: unknown, the
    // default lag is used), now = local seconds. The fix shows the robot `age + netDelay` ago.
    CamResult onCamera(float x, float y, float h, double tImage, float age, double now) {
        if (tImage > 0.0) {
            if (haveT_ && tImage <= lastT_) return CamResult::RejectedOrder;   // rule: duplicate / out of order
            haveT_ = true; lastT_ = tImage;
        }
        float lag = age >= 0.f ? age + P.camNetDelay : P.camDefaultLag;
        if (lag > P.camMaxAge) return CamResult::RejectedOld;    // rule: too old to compensate
        // rule (pose_fusion.m): move the fix forward by the odometry motion since the image (rigid motion in the
        // robot frame), so it is compared with the estimate at the SAME instant
        Pose o0 = odomAt(now - lag);
        float c0 = std::cos(o0.th), s0 = std::sin(o0.th);
        float dx = odom_.x - o0.x, dy = odom_.y - o0.y;
        float rx = c0 * dx + s0 * dy, ry = -s0 * dx + c0 * dy, dth = wrapAngle(odom_.th - o0.th);
        float hz = wrapAngle(h);
        float cz = std::cos(hz), sz = std::sin(hz);
        Pose z = makePose(x + cz * rx - sz * ry, y + sz * rx + cz * ry, wrapAngle(hz + dth));
        if (!camInit_) { est_ = z; camInit_ = true; return CamResult::Initialised; }  // rule: first fix sets the pose
        float ex = z.x - est_.x, ey = z.y - est_.y, eh = wrapAngle(z.th - est_.th);
        if (std::sqrt(ex * ex + ey * ey) > P.camMaxJump || std::fabs(eh) > P.camMaxJumpRad) {
            // rule: an implausible jump is rejected, unless the camera insists (robot moved by hand)
            if (++rejectRun_ >= P.camRelock) { est_ = z; rejectRun_ = 0; return CamResult::Relocked; }
            return CamResult::RejectedJump;
        }
        rejectRun_ = 0;
        // rule: complementary fusion est += gain * (z - est), heading wrapped (pose_fusion.m)
        est_.x += P.camGain * ex; est_.y += P.camGain * ey; est_.th = wrapAngle(est_.th + P.camGain * eh);
        return CamResult::Accepted;
    }

    // Bench: open-loop PWM for `seconds` (overrides the controller). A NEW command (onCommand) cancels it.
    void setRaw(int pwmL, int pwmR, double seconds, double now) {
        rawL_ = (float)pwmL; rawR_ = (float)pwmR; rawUntil_ = now + seconds; raw_ = seconds > 0.0;
    }
    void setPose(const Pose& p) { est_ = p; camInit_ = true; }

    // One control tick at time `now` (s) with the cumulative encoder counts.
    void step(double now, int32_t encL, int32_t encR) {
        const float dt = P.dt;
        int32_t dL = encL - lastEncL_, dR = encR - lastEncR_;
        est_ = odometry(est_, dL, dR, P);                        // est follows the encoders between fixes
        odom_ = odometry(odom_, dL, dR, P);                      // odom: encoders only (latency compensation)
        lastEncL_ = encL; lastEncR_ = encR;
        pushHistory(now, odom_);
        vL_ = spdL_.update(encL, P.mPerCountL(), P.velWindow, dt);
        vR_ = spdR_.update(encR, P.mPerCountR(), P.velWindow, dt);

        if (raw_) {
            if (now < rawUntil_) {
                pwmL_ = rawL_; pwmR_ = rawR_; refL_ = refR_ = 0.f; cmdL_ = cmdR_ = 0.f;
                piL_.reset(); piR_.reset();
                return;
            }
            raw_ = false;
        }

        // rule: "stopped" = both wheels below 2 mm/s and nothing commanded (fake_bot: max|v| < 2e-3)
        bool stopped = maxf(std::fabs(vL_), std::fabs(vR_)) < P.stoppedSpeed &&
                       std::fabs(refL_) < P.refZero && std::fabs(refR_) < P.refZero;

        // rule (watchdog): no cmd for 300 ms while driving / turning = stop the wheels, report idle, keep the goal,
        // resume when messages return (dump and braking carry on: the wheels are commanded to zero there anyway)
        bool active = (kind_ == Kind::Goto && phase_ == Phase::Drive) || (kind_ == Kind::Turn && phase_ == Phase::Turn);
        bool silent = !local_ && active && (now - lastRx_ > P.watchdog);
        WheelCmd cmd;
        if (silent) {
            stalled_ = true;
            state_ = State::Idle;
            rot_ = false;
        } else {
            if (stalled_) {
                stalled_ = false;
                if (kind_ == Kind::Goto) state_ = State::Moving;
                else if (kind_ == Kind::Turn) state_ = State::Turning;
            }
            cmd = control(stopped, dt);
        }

        // rule: wheel speed limit 0.534 m/s (both scaled: the curvature is kept)
        float m = maxf(std::fabs(cmd.vL), std::fabs(cmd.vR));
        if (m > P.maxWheel) { cmd.vL *= P.maxWheel / m; cmd.vR *= P.maxWheel / m; }
        cmdL_ = cmd.vL; cmdR_ = cmd.vR;
        // rule: acceleration limit 0.5 m/s^2 on each wheel speed reference
        float dv = P.accel * dt;
        refL_ += clampf(cmd.vL - refL_, -dv, dv);
        refR_ += clampf(cmd.vR - refR_, -dv, dv);
        pwmL_ = piL_.update(refL_, vL_, dt, P);
        pwmR_ = piR_.update(refR_, vR_, dt, P);
    }

    // ---- read-outs
    const Pose& est() const { return est_; }
    const Pose& odom() const { return odom_; }
    State state() const { return state_; }
    uint32_t seq() const { return seq_; }
    bool stalled() const { return stalled_; }
    bool rawActive() const { return raw_; }
    bool localActive() const { return local_; }                 // current command came from the serial bench
    bool camInitialised() const { return camInit_; }
    float pwmL() const { return pwmL_; }
    float pwmR() const { return pwmR_; }
    float refL() const { return refL_; }
    float refR() const { return refR_; }
    float cmdL() const { return cmdL_; }
    float cmdR() const { return cmdR_; }
    float speedL() const { return vL_; }
    float speedR() const { return vR_; }
    const Params& params() const { return P; }

private:
    enum class Kind : uint8_t { None, Goto, Turn, Dump, Stop };
    enum class Phase : uint8_t { None, Drive, Brake, Turn, Brake2, Dump, Stop };
    enum { H = 64 };                                            // odometry history: 64 ticks = 0.32 s at 200 Hz

    // fake_bot.py Bot.control: wheel speed command of the active command, advances the phases
    WheelCmd control(bool stopped, float dt) {
        bool arr = false;
        switch (kind_) {
            case Kind::Goto:
                if (phase_ == Phase::Drive) {
                    // rule (brain_step.m): steer along the LINE through the target with direction `heading`: the aim
                    // point lies toward a carrot `lookahead` ahead of the robot's foot point on that line, at the
                    // distance still to go + arriveTol; in the last slideDist the aim slides onto the robot's own
                    // track (brake straight along the leg)
                    float c = std::cos(h_), s = std::sin(h_);
                    float along = (tx_ - est_.x) * c + (ty_ - est_.y) * s;
                    float aim = along + P.arriveTol;
                    float ax, ay;
                    if (along < P.slide) {
                        float a = maxf(aim, 0.f);
                        ax = est_.x + a * c; ay = est_.y + a * s;
                    } else {
                        float fx = tx_ - along * c, fy = ty_ - along * s;     // foot point on the leg line
                        float la = minf(along, P.lookahead);
                        float cvx = fx + la * c - est_.x, cvy = fy + la * s - est_.y;
                        float n = std::sqrt(cvx * cvx + cvy * cvy);
                        if (n < 1e-9f) { cvx = c; cvy = s; n = 1.f; }
                        ax = est_.x + aim * cvx / n; ay = est_.y + aim * cvy / n;
                    }
                    WheelCmd w = gotoLaw(est_, ax, ay, speed_, P, arr, rot_);
                    // rule: brake when the predicted coast reaches the end point (stops centre on it)
                    float vs = 0.5f * (vL_ + vR_);
                    if (along <= coastDistance(vs, P) || (arr && along <= P.arriveTol)) {
                        phase_ = Phase::Brake; rot_ = false;
                        return WheelCmd();
                    }
                    return w;
                }
                // rule: a goto ends when the wheels have stopped; NO final turn (the brain plans every turn itself)
                if (phase_ == Phase::Brake && stopped) finish();
                return WheelCmd();
            case Kind::Turn:
                if (phase_ == Phase::Turn) {
                    WheelCmd w = turnLaw(est_.th, h_, dir_, tol_, P, arr);
                    if (arr) { phase_ = Phase::Brake2; return WheelCmd(); }
                    return w;
                }
                if (phase_ == Phase::Brake2 && stopped) finish();    // rule: arrived only with the wheels stopped
                return WheelCmd();
            case Kind::Dump:
                timer_ += dt;                                    // rule: wheels still for dumpTime, then arrived
                if (timer_ >= P.dumpTime) finish();
                return WheelCmd();
            case Kind::Stop:
                if (stopped) finish();                           // rule: stop = wheels to zero, arrived once stopped
                return WheelCmd();
            default:
                return WheelCmd();
        }
    }
    void finish() { kind_ = Kind::None; phase_ = Phase::None; state_ = State::Arrived; }

    void pushHistory(double t, const Pose& p) {
        histHead_ = (histHead_ + 1) % H;
        histT_[histHead_] = t; hist_[histHead_] = p;
        if (histFilled_ < H) ++histFilled_;
    }
    // odometry pose at time t, linearly interpolated (hist_at.m); clamped to the oldest / newest entry
    Pose odomAt(double t) const {
        int k = histHead_;
        if (t >= histT_[k]) return hist_[k];
        for (int n = 1; n < histFilled_; ++n) {
            int j = (histHead_ - n + H) % H;
            if (histT_[j] <= t) {
                const Pose& a = hist_[j]; const Pose& b = hist_[k];
                double span = histT_[k] - histT_[j];
                float f = span > 0.0 ? (float)((t - histT_[j]) / span) : 0.f;
                return makePose(a.x + f * (b.x - a.x), a.y + f * (b.y - a.y), wrapAngle(a.th + f * wrapAngle(b.th - a.th)));
            }
            k = j;
        }
        return hist_[k];                                         // older than the history: the oldest entry
    }

    Params P;
    Pose est_, odom_;
    Pose hist_[H];
    double histT_[H];
    int histHead_ = 0, histFilled_ = 1;
    int32_t lastEncL_ = 0, lastEncR_ = 0;
    SpeedEstimator spdL_, spdR_;
    WheelPI piL_, piR_;
    float vL_ = 0.f, vR_ = 0.f, refL_ = 0.f, refR_ = 0.f, cmdL_ = 0.f, cmdR_ = 0.f, pwmL_ = 0.f, pwmR_ = 0.f;

    State state_ = State::Idle;
    uint32_t seq_ = 0;
    Kind kind_ = Kind::None;
    Phase phase_ = Phase::None;
    float tx_ = 0.f, ty_ = 0.f, h_ = 0.f, speed_ = CRUISE_SPEED, timer_ = 0.f, tol_ = 0.f;
    int dir_ = 0;
    bool local_ = false, stalled_ = false, rot_ = false;
    double lastRx_ = -1e9;

    bool camInit_ = false, haveT_ = false;
    double lastT_ = 0.0;
    int rejectRun_ = 0;

    bool raw_ = false;
    float rawL_ = 0.f, rawR_ = 0.f;
    double rawUntil_ = 0.0;
};

}  // namespace zadp
