// mirror_test.js : a line-by-line JavaScript mirror of include/motion_core.h and test/host/test_motion_core.cpp.
//
// WHY: the machine this firmware was written on had no host C++ compiler (no g++, clang or MSVC; only the
// ESP32 cross compiler, which can compile but not run the test). Node.js was available, so the same maths and
// the same simulated robot were transcribed here to get numbers. Differences to the C++: double precision
// instead of float, a different random generator. Keep both files in step with motion_core.h.
//
// Run: node test/host/js_mirror/mirror_test.js     (exit code 0 = all checks passed)
'use strict';

// ---------------------------------------------------------------- config.h numbers
const C = {
  WHEEL_DIAMETER_M: 0.034, TRACK_M: 0.0753, CPR_LEFT: 2772.1, CPR_RIGHT: 2772.3, TRACK_SCALE: 1.0,
  MAX_WHEEL_SPEED: 0.53407, CRUISE_SPEED: 0.20, MIN_GOTO_SPEED: 0.02, MAX_ACCEL: 0.5, MOTOR_TAU_S: 0.08,
  KP_HEADING: 3.0, KP_DISTANCE: 2.5, ROTATE_THRESH_DEG: 25, ROTATE_EXIT_DEG: 15, DECEL_FRAC: 0.5, ARRIVE_TOL_M: 0.005,
  HEADING_TOL_DEG: 3, MIN_TURN_RATE: 0.3, CAPTURE_RADIUS_M: 0.03, SLIDE_DIST_M: 0.05, LOOKAHEAD_M: 0.15,
  TURN_WRAP_SLACK: 0.3, STOPPED_SPEED: 0.002, DUMP_TIME_S: 4.0, WATCHDOG_S: 0.3,
  CAM_FUSION_GAIN: 0.3, CAM_NET_DELAY_S: 0.01, CAM_DEFAULT_LAG_S: 0.05, CAM_MAX_AGE_S: 0.25,
  CAM_MAX_JUMP_M: 0.05, CAM_MAX_JUMP_DEG: 20, CAM_RELOCK_COUNT: 10,
  CONTROL_HZ: 200, VEL_WINDOW: 4, PWM_MAX: 255, PWM_DEADBAND: 15,
  WHEEL_KP: 600, WHEEL_KI: 7500, WHEEL_KD: 0, WHEEL_I_LIMIT: 120, REF_ZERO: 1e-4,
};
C.PWM_PER_MPS = 432.6;

const kPi = Math.PI, kTwoPi = 2 * Math.PI;
const wrapAngle = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const clampf = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
const sgnf = (v) => (v > 0 ? 1 : v < 0 ? -1 : 0);
const deg2rad = (d) => (d * kPi) / 180;
const rad2deg = (r) => (r * 180) / kPi;
const pyMod = (a, m) => a - m * Math.floor(a / m);

function Params() {
  const P = {
    wheelD: C.WHEEL_DIAMETER_M, cprL: C.CPR_LEFT, cprR: C.CPR_RIGHT, track: C.TRACK_M * C.TRACK_SCALE,
    maxWheel: C.MAX_WHEEL_SPEED, cruise: C.CRUISE_SPEED, minGoto: C.MIN_GOTO_SPEED, accel: C.MAX_ACCEL, motorTau: C.MOTOR_TAU_S,
    kpHead: C.KP_HEADING, kpDist: C.KP_DISTANCE, rotThresh: deg2rad(C.ROTATE_THRESH_DEG), rotExit: deg2rad(C.ROTATE_EXIT_DEG),
    decelFrac: C.DECEL_FRAC, arriveTol: C.ARRIVE_TOL_M, headTol: deg2rad(C.HEADING_TOL_DEG), minTurnRate: C.MIN_TURN_RATE,
    capture: C.CAPTURE_RADIUS_M, slide: C.SLIDE_DIST_M, lookahead: C.LOOKAHEAD_M, turnSlack: C.TURN_WRAP_SLACK,
    stoppedSpeed: C.STOPPED_SPEED, dumpTime: C.DUMP_TIME_S, watchdog: C.WATCHDOG_S,
    camGain: C.CAM_FUSION_GAIN, camNetDelay: C.CAM_NET_DELAY_S, camDefaultLag: C.CAM_DEFAULT_LAG_S, camMaxAge: C.CAM_MAX_AGE_S,
    camMaxJump: C.CAM_MAX_JUMP_M, camMaxJumpRad: deg2rad(C.CAM_MAX_JUMP_DEG), camRelock: C.CAM_RELOCK_COUNT,
    dt: 1 / C.CONTROL_HZ, velWindow: C.VEL_WINDOW,
    pwmMax: C.PWM_MAX, deadband: C.PWM_DEADBAND, pwmPerMps: C.PWM_PER_MPS,
    kp: C.WHEEL_KP, ki: C.WHEEL_KI, kd: C.WHEEL_KD, iLimit: C.WHEEL_I_LIMIT, refZero: C.REF_ZERO,
  };
  P.mPerCountL = kPi * P.wheelD / P.cprL;
  P.mPerCountR = kPi * P.wheelD / P.cprR;
  return P;
}

const pose = (x, y, th) => ({ x, y, th });
// chord form of the exact arc (motion_core.h arcStep). R = rounding per operation: identity (double) or
// Math.fround (float32 emulation of the ESP32 build, used by testArcFloat32).
function arcStep(p, ds, dth, R = (v) => v) {
  const h = R(0.5 * dth);
  const k = Math.abs(h) < 1e-3 ? R(1 - R(R(h * h) / 6)) : R(R(Math.sin(h)) / h);
  const dk = R(ds * k), a = R(p.th + h);
  return { x: R(p.x + R(dk * R(Math.cos(a)))), y: R(p.y + R(dk * R(Math.sin(a)))), th: R(p.th + dth) };
}
// the previous difference-of-sines form, kept only to show that testArcFloat32 detects its float32 failure
function arcStepOld(p, ds, dth, R) {
  const q = { x: p.x, y: p.y, th: p.th };
  if (Math.abs(dth) < 1e-9) { q.x = R(q.x + R(ds * R(Math.cos(p.th)))); q.y = R(q.y + R(ds * R(Math.sin(p.th)))); }
  else { const r = R(ds / dth); const a = R(p.th + dth);
    q.x = R(q.x + R(r * R(R(Math.sin(a)) - R(Math.sin(p.th))))); q.y = R(q.y - R(r * R(R(Math.cos(a)) - R(Math.cos(p.th))))); }
  q.th = R(p.th + dth);
  return q;
}
function odometry(p, dL, dR, P) {
  const sl = dL * P.mPerCountL, sr = dR * P.mPerCountR;
  const q = arcStep(p, 0.5 * (sl + sr), (sr - sl) / P.track);
  q.th = wrapAngle(q.th);
  return q;
}
class SpeedEstimator {
  constructor() { this.N = 32; this.hist = new Array(32).fill(0); this.head = 0; this.filled = 0; }
  update(count, mpc, window, dt) {
    const N = this.N;
    this.hist[this.head] = count;
    let w = window < this.filled ? window : this.filled;
    if (w > N - 1) w = N - 1;
    let v = 0;
    if (w > 0) v = (count - this.hist[(this.head - w + N) % N]) * mpc / (w * dt);
    this.head = (this.head + 1) % N;
    if (this.filled < N - 1) this.filled++;
    return v;
  }
}
class WheelPI {
  constructor() { this.integ = 0; this.prevErr = 0; }
  reset() { this.integ = 0; this.prevErr = 0; }
  static ff(v, P) { if (Math.abs(v) < P.refZero) return 0; return sgnf(v) * (P.deadband + Math.abs(v) * P.pwmPerMps); }
  update(ref, meas, dt, P) {
    if (Math.abs(ref) < P.refZero) { this.reset(); return 0; }
    const e = ref - meas, d = (e - this.prevErr) / dt;
    this.prevErr = e;
    const uNoI = WheelPI.ff(ref, P) + P.kp * e + P.kd * d;
    const iNew = clampf(this.integ + P.ki * e * dt, -P.iLimit, P.iLimit);
    let u = uNoI + iNew;
    if ((u > P.pwmMax && e > 0) || (u < -P.pwmMax && e < 0)) u = uNoI + this.integ; else this.integ = iNew;
    return clampf(u, -P.pwmMax, P.pwmMax);
  }
}
const mk = (v, w, b) => ({ vL: v - w * b / 2, vR: v + w * b / 2 });
// returns {cmd, arr, rot}; rot = rotate-in-place mode in/out (hysteresis)
function gotoLaw(pe, tx, ty, cruise, P, rot) {
  const dx = tx - pe.x, dy = ty - pe.y, d = Math.hypot(dx, dy);
  if (d < P.arriveTol) return { cmd: { vL: 0, vR: 0 }, arr: true, rot: false };
  const e = wrapAngle(Math.atan2(dy, dx) - pe.th);
  if (d < P.capture) rot = false; else if (rot) rot = Math.abs(e) > P.rotExit; else rot = Math.abs(e) > P.rotThresh;
  let v, w;
  if (rot) { const wmax = cruise / (P.track / 2); w = clampf(P.kpHead * e, -wmax, wmax); v = 0; }
  else {
    v = Math.min(cruise, P.kpDist * d, Math.sqrt(2 * P.decelFrac * P.accel * d)) * Math.cos(e);
    let eS = e; if (Math.cos(e) < 0) eS = wrapAngle(e + kPi);
    w = P.kpHead * eS * Math.min(1, d / P.capture);
  }
  return { cmd: mk(v, w, P.track), arr: false, rot };
}
function turnLaw(th, h, dir, tol, P) {
  if (dir === 0) dir = wrapAngle(h - th) >= 0 ? 1 : -1;
  const e = pyMod(dir * (h - th) + P.turnSlack, kTwoPi) - P.turnSlack;
  if (Math.abs(e) < tol) return { cmd: { vL: 0, vR: 0 }, arr: true };
  const wmax = P.cruise / (P.track / 2);
  const w = dir * sgnf(e) * Math.min(wmax, Math.max(P.minTurnRate, P.kpHead * Math.abs(e)));
  return { cmd: mk(0, w, P.track), arr: false };
}
function coastDistance(v, P) {
  v = Math.max(v, 0); const a = P.accel, tau = P.motorTau;
  return v <= a * tau ? v * tau : v * v / (2 * a) + a * tau * tau / 2;
}

const A = { Unknown: 0, Goto: 1, Turn: 2, Dump: 3, Stop: 4 };
const S = { Idle: 'idle', Moving: 'moving', Turning: 'turning', Dumping: 'dumping', Arrived: 'arrived' };
const CR = { Accepted: 'acc', Initialised: 'init', Relocked: 'relock', RejectedJump: 'jump', RejectedOld: 'old', RejectedOrder: 'order' };
const H = 64;

class Truck {
  constructor(P) { this.P = P || Params(); this.camInit = false; this.haveT = false; this.lastT = 0; this.rejectRun = 0;
    this.raw = false; this.lastRx = -1e9; this.local = false; this.stalled = false; this.rot = false; this.speed = C.CRUISE_SPEED;
    this.timer = 0; this.dir = 0; this.tol = 0; }
  begin(start, eL, eR, now = 0) {
    this.est = { ...start }; this.odom = { ...start }; this.lastEncL = eL; this.lastEncR = eR;
    this.hist = []; this.histT = []; for (let i = 0; i < H; i++) { this.hist.push({ ...start }); this.histT.push(now); }
    this.histHead = 0; this.histFilled = 1;
    this.state = S.Idle; this.seq = 0; this.kind = 'none'; this.phase = 'none'; this.stalled = false; this.rot = false;
    this.refL = this.refR = 0; this.pwmL = this.pwmR = 0; this.cmdL = this.cmdR = 0; this.vL = this.vR = 0;
    this.spdL = new SpeedEstimator(); this.spdR = new SpeedEstimator(); this.piL = new WheelPI(); this.piR = new WheelPI();
  }
  onCommand(c, now) {
    const P = this.P;
    this.lastRx = now;
    if (c.seq === this.seq && c.seq !== 0) return false;
    this.seq = c.seq; this.raw = false; this.stalled = false; this.local = !!c.local; this.rot = false;
    let speed = c.speed > 0 ? c.speed : P.cruise; speed = Math.min(speed, P.cruise);
    let a = c.action;
    if ((a === A.Goto && !c.hasXY) || (a === A.Turn && !c.hasHeading)) a = A.Unknown;
    if (a === A.Goto) { this.kind = 'goto'; this.phase = 'drive'; this.tx = c.x; this.ty = c.y;
      this.h = c.hasHeading ? c.heading : Math.atan2(c.y - this.est.y, c.x - this.est.x); this.speed = Math.max(speed, P.minGoto); this.state = S.Moving; }
    else if (a === A.Turn) { this.kind = 'turn'; this.phase = 'turn'; this.h = c.heading; this.dir = c.dir > 0 ? 1 : c.dir < 0 ? -1 : 0;
      this.tol = c.tolDeg > 0 ? deg2rad(c.tolDeg) : P.headTol; this.state = S.Turning; }
    else if (a === A.Dump) { this.kind = 'dump'; this.phase = 'dump'; this.timer = 0; this.state = S.Dumping; }
    else if (a === A.Stop) { this.kind = 'stop'; this.phase = 'stop'; this.state = S.Idle; }
    else { this.kind = 'none'; this.phase = 'none'; this.state = S.Idle; }
    return true;
  }
  onCamera(x, y, h, tImage, age, now) {
    const P = this.P;
    if (tImage > 0) {
      if (this.haveT && tImage <= this.lastT) return CR.RejectedOrder;
      this.haveT = true; this.lastT = tImage;
    }
    const lag = age >= 0 ? age + P.camNetDelay : P.camDefaultLag;
    if (lag > P.camMaxAge) return CR.RejectedOld;
    const o0 = this.odomAt(now - lag), o1 = this.odom;
    const c0 = Math.cos(o0.th), s0 = Math.sin(o0.th), dx = o1.x - o0.x, dy = o1.y - o0.y;
    const rx = c0 * dx + s0 * dy, ry = -s0 * dx + c0 * dy, dth = wrapAngle(o1.th - o0.th);
    const hz = wrapAngle(h), cz = Math.cos(hz), sz = Math.sin(hz);
    const z = pose(x + cz * rx - sz * ry, y + sz * rx + cz * ry, wrapAngle(hz + dth));
    if (!this.camInit) { this.est = { ...z }; this.camInit = true; return CR.Initialised; }
    const ex = z.x - this.est.x, ey = z.y - this.est.y, eh = wrapAngle(z.th - this.est.th);
    if (Math.hypot(ex, ey) > P.camMaxJump || Math.abs(eh) > P.camMaxJumpRad) {
      if (++this.rejectRun >= P.camRelock) { this.est = { ...z }; this.rejectRun = 0; return CR.Relocked; }
      return CR.RejectedJump;
    }
    this.rejectRun = 0;
    this.est.x += P.camGain * ex; this.est.y += P.camGain * ey; this.est.th = wrapAngle(this.est.th + P.camGain * eh);
    return CR.Accepted;
  }
  setPose(p) { this.est = { ...p }; this.camInit = true; }
  step(now, encL, encR) {
    const P = this.P, dt = P.dt;
    const dL = encL - this.lastEncL, dR = encR - this.lastEncR;
    this.est = odometry(this.est, dL, dR, P);
    this.odom = odometry(this.odom, dL, dR, P);
    this.lastEncL = encL; this.lastEncR = encR;
    this.pushHistory(now, this.odom);
    this.vL = this.spdL.update(encL, P.mPerCountL, P.velWindow, dt);
    this.vR = this.spdR.update(encR, P.mPerCountR, P.velWindow, dt);
    const stopped = Math.max(Math.abs(this.vL), Math.abs(this.vR)) < P.stoppedSpeed && Math.abs(this.refL) < P.refZero && Math.abs(this.refR) < P.refZero;
    const active = (this.kind === 'goto' && this.phase === 'drive') || (this.kind === 'turn' && this.phase === 'turn');
    const silent = !this.local && active && now - this.lastRx > P.watchdog;
    let cmd = { vL: 0, vR: 0 };
    if (silent) { this.stalled = true; this.state = S.Idle; this.rot = false; }
    else {
      if (this.stalled) { this.stalled = false; if (this.kind === 'goto') this.state = S.Moving; else if (this.kind === 'turn') this.state = S.Turning; }
      cmd = this.control(stopped, dt);
    }
    const m = Math.max(Math.abs(cmd.vL), Math.abs(cmd.vR));
    if (m > P.maxWheel) { cmd.vL *= P.maxWheel / m; cmd.vR *= P.maxWheel / m; }
    this.cmdL = cmd.vL; this.cmdR = cmd.vR;
    const dv = P.accel * dt;
    this.refL += clampf(cmd.vL - this.refL, -dv, dv);
    this.refR += clampf(cmd.vR - this.refR, -dv, dv);
    this.pwmL = this.piL.update(this.refL, this.vL, dt, P);
    this.pwmR = this.piR.update(this.refR, this.vR, dt, P);
  }
  control(stopped, dt) {
    const P = this.P, Z = { vL: 0, vR: 0 };
    if (this.kind === 'goto') {
      if (this.phase === 'drive') {
        const c = Math.cos(this.h), s = Math.sin(this.h), e = this.est;
        const along = (this.tx - e.x) * c + (this.ty - e.y) * s, aim = along + P.arriveTol;
        let ax, ay;
        if (along < P.slide) { const a = Math.max(aim, 0); ax = e.x + a * c; ay = e.y + a * s; }
        else {
          const fx = this.tx - along * c, fy = this.ty - along * s, la = Math.min(along, P.lookahead);
          let cvx = fx + la * c - e.x, cvy = fy + la * s - e.y, n = Math.hypot(cvx, cvy);
          if (n < 1e-9) { cvx = c; cvy = s; n = 1; }
          ax = e.x + aim * cvx / n; ay = e.y + aim * cvy / n;
        }
        const r = gotoLaw(e, ax, ay, this.speed, P, this.rot); this.rot = r.rot;
        const vs = 0.5 * (this.vL + this.vR);
        if (along <= coastDistance(vs, P) || (r.arr && along <= P.arriveTol)) { this.phase = 'brake'; this.rot = false; return Z; }
        return r.cmd;
      }
      if (this.phase === 'brake' && stopped) this.finish();
      return Z;
    }
    if (this.kind === 'turn') {
      if (this.phase === 'turn') {
        const r = turnLaw(this.est.th, this.h, this.dir, this.tol, P);
        if (r.arr) { this.phase = 'brake2'; return Z; }
        return r.cmd;
      }
      if (this.phase === 'brake2' && stopped) this.finish();
      return Z;
    }
    if (this.kind === 'dump') { this.timer += dt; if (this.timer >= P.dumpTime) this.finish(); return Z; }
    if (this.kind === 'stop') { if (stopped) this.finish(); return Z; }
    return Z;
  }
  finish() { this.kind = 'none'; this.phase = 'none'; this.state = S.Arrived; }
  pushHistory(t, p) { this.histHead = (this.histHead + 1) % H; this.histT[this.histHead] = t; this.hist[this.histHead] = { ...p }; if (this.histFilled < H) this.histFilled++; }
  odomAt(t) {
    let k = this.histHead;
    if (t >= this.histT[k]) return this.hist[k];
    for (let n = 1; n < this.histFilled; n++) {
      const j = (this.histHead - n + H) % H;
      if (this.histT[j] <= t) {
        const a = this.hist[j], b = this.hist[k], span = this.histT[k] - this.histT[j];
        const f = span > 0 ? (t - this.histT[j]) / span : 0;
        return pose(a.x + f * (b.x - a.x), a.y + f * (b.y - a.y), wrapAngle(a.th + f * wrapAngle(b.th - a.th)));
      }
      k = j;
    }
    return this.hist[k];
  }
}

// ---------------------------------------------------------------- simulation (mirror of the C++ test)
function rng(seed) {   // mulberry32 + Box-Muller
  let a = seed >>> 0;
  const u = () => { a = (a + 0x6D2B79F5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
  const n = () => { let x = 0; while (x === 0) x = u(); return Math.sqrt(-2 * Math.log(x)) * Math.cos(2 * Math.PI * u()); };
  return { u, n };
}
class Plant {
  constructor(seed) {
    this.r = rng(seed); this.pose = pose(0, 0, 0); this.wheel = [0, 0]; this.v = [0, 0];
    this.deadband = 28; this.vmaxLoaded = 0.48; this.tau = 0.08; this.tauBrake = 0.03;
    this.wheelD = [C.WHEEL_DIAMETER_M, C.WHEEL_DIAMETER_M]; this.track = C.TRACK_M; this.cpr = [2772.1, 2772.3];
    this.slipStd = 0.01; this.slipCorr = 0.2; this.slip = [1 + 0.01 * this.r.n(), 1 + 0.01 * this.r.n()];
    this.histT = []; this.histP = [];
  }
  enc(i) { return Math.floor(this.wheel[i] / (2 * Math.PI) * this.cpr[i]); }
  step(pwm, dt) {
    const ds = [0, 0];
    for (let i = 0; i < 2; i++) {
      const u = pwm[i]; let vss = 0, t = this.tau;
      if (u === 0) t = this.tauBrake;
      else if (Math.abs(u) > this.deadband) vss = sgnf(u) * (Math.abs(u) - this.deadband) / (255 - this.deadband) * this.vmaxLoaded;
      this.v[i] += (vss - this.v[i]) * (1 - Math.exp(-dt / t));
      const dRot = this.v[i] * dt;
      this.wheel[i] += dRot / (C.WHEEL_DIAMETER_M / 2);
      const a = Math.exp(-Math.abs(dRot) / this.slipCorr);
      this.slip[i] = 1 + (this.slip[i] - 1) * a + this.slipStd * Math.sqrt(1 - a * a) * this.r.n();
      ds[i] = dRot * (this.wheelD[i] / C.WHEEL_DIAMETER_M) * this.slip[i];
    }
    this.pose = arcStep(this.pose, (ds[0] + ds[1]) / 2, (ds[1] - ds[0]) / this.track);
    this.pose.th = wrapAngle(this.pose.th);
  }
}
// camera: image at tImage, published camAge later (the age field), arrives 5-15 ms after that (network)
class Harness {
  constructor(seed, start, P) {
    this.plant = new Plant(seed); this.truck = new Truck(P || Params()); this.dt = this.truck.P.dt; this.sub = 5; this.t = 0;
    this.camOn = true; this.keepAlive = true; this.last = { seq: 0 }; this.nextRepeat = 0; this.nextCam = 0; this.camQ = [];
    this.clockOffset = 1795023945.0; this.maxRef = 0; this.maxCmd = 0; this.camAge = 0.04; this.ageReported = true;
    this.camNoise = true; this.past = [];
    this.plant.pose = { ...start }; this.truck.begin(start, 0, 0, 0);
  }
  send(c) { this.last = c; this.nextRepeat = this.t + 0.1; return this.truck.onCommand(c, this.t); }
  tick() {
    const T = this.truck, p = this.plant;
    this.past.push({ t: this.t, p: { ...p.pose } }); if (this.past.length > 100) this.past.shift();
    if (this.keepAlive && this.last.seq !== 0 && this.t >= this.nextRepeat) { T.onCommand(this.last, this.t); this.nextRepeat += 0.1; }
    if (this.t >= this.nextCam) {
      this.nextCam += 1 / 30;
      if (this.camOn) {
        const sx = this.camNoise ? 0.003 : 0, sh = this.camNoise ? deg2rad(1) : 0;
        this.camQ.push({ tImage: this.t + this.clockOffset, age: this.ageReported ? this.camAge : -1, deliver: this.t + this.camAge + 0.005 + 0.01 * p.r.u(),
          x: p.pose.x + sx * p.r.n(), y: p.pose.y + sx * p.r.n(), h: wrapAngle(p.pose.th + sh * p.r.n()) });
      }
    }
    while (this.camQ.length && this.camQ[0].deliver <= this.t) { const m = this.camQ.shift(); T.onCamera(m.x, m.y, m.h, m.tImage, m.age, this.t); }
    T.step(this.t, p.enc(0), p.enc(1));
    this.maxRef = Math.max(this.maxRef, Math.abs(T.refL), Math.abs(T.refR));
    this.maxCmd = Math.max(this.maxCmd, Math.abs(T.cmdL), Math.abs(T.cmdR));
    const pwm = [Math.round(T.pwmL), Math.round(T.pwmR)];
    for (let k = 0; k < this.sub; k++) p.step(pwm, this.dt / this.sub);
    this.t += this.dt;
  }
  runUntilArrived(seq, timeout) { const t0 = this.t; while (this.t - t0 < timeout) { this.tick(); if (this.truck.state === S.Arrived && this.truck.seq === seq) return this.t - t0; } return -1; }
  run(s) { const t0 = this.t; while (this.t - t0 < s) this.tick(); }
  trueSpeed() { return Math.max(Math.abs(this.plant.v[0]), Math.abs(this.plant.v[1])); }
}
const gotoCmd = (seq, x, y, h) => ({ seq, action: A.Goto, x, y, heading: h, hasXY: true, hasHeading: true, speed: 0.2, dir: 0, tolDeg: 0 });
const turnCmd = (seq, h, dir, tolDeg = 0) => ({ seq, action: A.Turn, heading: h, hasHeading: true, dir, speed: 0.2, tolDeg });
const simple = (seq, a) => ({ seq, action: a, speed: 0, dir: 0, tolDeg: 0 });
const dist = (a, x, y) => Math.hypot(a.x - x, a.y - y);
// signed distance of p from the line through (x, y) with direction h
const cross = (p, x, y, h) => -(p.x - x) * Math.sin(h) + (p.y - y) * Math.cos(h);

let pass = 0, fail = 0;
const check = (ok, what) => { ok ? pass++ : fail++; console.log(`  [${ok ? 'PASS' : 'FAIL'}] ${what}`); };
const f = (v, n = 2) => v.toFixed(n);

function testSemantics() {
  console.log('\nTEST seq / state semantics (TOPICS.md, fake_bot.py)');
  const Hh = new Harness(601, pose(0.2, 0.5, 0)); const T = Hh.truck;
  check(T.seq === 0 && T.state === S.Idle, 'before the first command: seq 0, idle');
  let ack = Hh.send(gotoCmd(1, 0.5, 0.5, 0));
  check(ack && T.state === S.Moving && T.seq === 1, 'goto seq 1: new (ack), moving');
  check(!T.onCommand(gotoCmd(1, 0.5, 0.5, 0), Hh.t), 'same seq again = repeat (no ack)');
  let t0 = Hh.t, vAt = -1, sawMoving = false;
  while (Hh.t - t0 < 10) { Hh.tick(); if (T.state === S.Moving) sawMoving = true; if (T.state === S.Arrived) { vAt = Math.max(Math.abs(T.vL), Math.abs(T.vR)); break; } }
  console.log(`  goto arrived after ${f(Hh.t - t0)} s, measured wheel speed at arrival ${f(vAt, 4)} m/s (true ${f(Hh.trueSpeed(), 4)})`);
  check(sawMoving && vAt >= 0 && vAt < C.STOPPED_SPEED && Hh.trueSpeed() < 0.003, 'arrived only with the wheels stopped');
  Hh.run(0.5);
  check(T.state === S.Arrived && T.seq === 1, "repeats of a finished command keep 'arrived' (no restart)");
  // goto whose heading differs from the robot's: NO final turn (TOPICS.md)
  Hh.send(gotoCmd(2, 0.6, 0.55, 0.4)); let sawTurning = false; t0 = Hh.t;
  while (Hh.t - t0 < 10) { Hh.tick(); if (T.state === S.Turning) sawTurning = true; if (T.state === S.Arrived) break; }
  console.log(`  goto ends: turning state seen ${sawTurning}, final heading ${f(rad2deg(T.est.th))} deg (command heading 22.9 deg)`);
  check(!sawTurning && T.state === S.Arrived, 'a goto never turns in place at its end');
  Hh.send(turnCmd(3, -1.5, 1)); check(T.state === S.Turning, 'turn: state turning');
  check(Hh.runUntilArrived(3, 10) > 0, 'turn arrives');
  Hh.send(turnCmd(31, 0.3, -1, 1)); const t1 = Hh.runUntilArrived(31, 10); const e1 = Math.abs(rad2deg(wrapAngle(T.est.th - 0.3)));
  console.log(`  turn with tol 1 deg: heading error ${f(e1)} deg (estimate)`);
  check(t1 > 0 && e1 < 1.0, 'a turn with `tol` 1 deg stops within 1 deg');
  Hh.keepAlive = false; Hh.send(simple(4, A.Dump)); check(T.state === S.Dumping, 'dump: state dumping');
  t0 = Hh.t; let tArr = -1, still = true;
  while (Hh.t - t0 < 6) { Hh.tick(); if (T.pwmL !== 0 || T.pwmR !== 0) still = false; if (T.state === S.Arrived) { tArr = Hh.t - t0; break; } }
  console.log(`  dump arrived after ${f(tArr, 3)} s (dumpTime 4.0)`);
  check(Math.abs(tArr - 4) < 0.011 && still, 'dump: wheels still for 4 s, then arrived');
  Hh.keepAlive = true; Hh.send(gotoCmd(5, 0.2, 0.5, kPi)); Hh.run(1.5);
  check(T.state === S.Moving, 'goto seq 5 under way');
  ack = Hh.send(simple(6, A.Stop)); check(ack && T.state === S.Idle && T.seq === 6, 'stop: new (ack), idle at once');
  const ts = Hh.t; const tStop = Hh.runUntilArrived(6, 2);
  console.log(`  stop: arrived ${f(tStop, 2)} s after the stop (true wheel speed ${f(Hh.trueSpeed(), 4)} m/s)`);
  check(tStop > 0 && Hh.trueSpeed() < 0.003, "stop: 'arrived' with the stop's seq once the wheels stopped");
  Hh.send(gotoCmd(7, 0.8, 0.5, 0)); Hh.run(1.0); Hh.send(gotoCmd(8, 0.3, 0.3, Math.atan2(0.3 - T.est.y, 0.3 - T.est.x)));
  let tt = Hh.runUntilArrived(8, 20);
  console.log(`  pre-empted goto seq 7 by seq 8: arrived at seq 8's target ${f(dist(T.est, 0.3, 0.3) * 1000, 1)} mm away`);
  check(tt > 0 && dist(T.est, 0.3, 0.3) < 0.0101, 'a different seq replaces the active command');
  ack = T.onCommand(simple(3, A.Dump), Hh.t); check(ack && T.state === S.Dumping && T.seq === 3, 'seq is not an order: a lower seq is a new command');
  ack = Hh.send(simple(9, A.Unknown)); check(ack && T.state === S.Idle && T.seq === 9, 'unknown action: accepted seq, idle');
  const l0 = gotoCmd(0, 0.4, 0.4, 0); const a1 = T.onCommand(l0, Hh.t), a2 = T.onCommand(l0, Hh.t);
  check(a1 && a2 && T.seq === 0, 'seq 0 (first-draft message): every message counts as new');
  const nh = gotoCmd(10, T.est.x + 0.1, T.est.y, 0); nh.hasHeading = false; Hh.send(nh);
  check(Hh.runUntilArrived(10, 10) > 0, 'goto without heading arrives (line from here to the target)');
  const big = gotoCmd(4000000000, T.est.x + 0.05, T.est.y, 0); check(Hh.send(big) && T.seq === 4000000000, 'seq above 2^31 accepted (uint32)');
  // watchdog stop, then a NEW dump: dumping (the stall flag is cleared; fake_bot.py had reported turning)
  const W = new Harness(602, pose(0.2, 0.5, 0)); W.camOn = false; W.send(gotoCmd(1, 0.8, 0.5, 0)); W.run(0.5); W.keepAlive = false; W.run(0.6);
  const wasIdle = W.truck.state === S.Idle && W.truck.stalled; W.keepAlive = true; W.send(simple(2, A.Dump)); W.tick();
  check(wasIdle && W.truck.state === S.Dumping && !W.truck.stalled, 'after a watchdog stop a new dump reports dumping');
}

function testGoto(cam) {
  const n = 40; console.log(`\nTEST goto from random starts (${n} runs, camera ${cam ? 'on' : 'off'})`);
  const r = rng(cam ? 11 : 12); const U = () => 0.1 + 0.8 * r.u(), Ang = () => -kPi + kTwoPi * r.u();
  let ok = 0, wE = 0, wT = 0, wX = 0, sum = 0, mx = 0, mref = 0;
  for (let k = 0; k < n; k++) {
    const s = pose(U(), U(), Ang()); let tx, ty; do { tx = U(); ty = U(); } while (Math.hypot(tx - s.x, ty - s.y) < 0.05);
    const h = Math.atan2(ty - s.y, tx - s.x);
    const Hh = new Harness(100 + k, s); Hh.camOn = cam; Hh.run(0.2); Hh.send(gotoCmd(1, tx, ty, h));
    const tt = Hh.runUntilArrived(1, 30);
    const eE = dist(Hh.truck.est, tx, ty), eT = dist(Hh.plant.pose, tx, ty), eX = Math.abs(cross(Hh.plant.pose, tx, ty, h));
    const good = tt > 0 && eE <= 0.0101 && Hh.trueSpeed() < 0.005;
    ok += good; wE = Math.max(wE, eE); wT = Math.max(wT, eT); wX = Math.max(wX, eX); mref = Math.max(mref, Hh.maxRef);
    if (tt > 0) { sum += tt; mx = Math.max(mx, tt); }
    if (!good) console.log(`    run ${k}: t=${f(tt)} eEst=${f(eE, 4)} eTrue=${f(eT, 4)}`);
  }
  console.log(`  arrived ${ok}/${n}, mean time ${f(sum / n)} s (max ${f(mx)}), worst |est-target| ${f(wE * 1000, 1)} mm, worst |true-target| ${f(wT * 1000, 1)} mm, worst TRUE cross-track ${f(wX * 1000, 1)} mm, max wheel ref ${f(mref, 3)} m/s`);
  check(ok === n, 'every goto arrives (estimate within 10 mm, wheels stopped)');
  if (cam) check(wT < 0.010, 'with the camera the TRUE position is within 10 mm of the target');
}

function lineRuns(y0, n) {
  let wX = 0, wAll = 0, wMid = 0;
  for (let k = 0; k < n; k++) {
    const Hh = new Harness(700 + k, pose(0.2, y0, 0)); Hh.run(0.2); Hh.send(gotoCmd(1, 0.8, 0.5, 0));
    let t0 = Hh.t, mid = 0, all = 0;
    while (Hh.t - t0 < 10) { Hh.tick(); const c = Math.abs(cross(Hh.plant.pose, 0.8, 0.5, 0)); all = Math.max(all, c); if (Hh.plant.pose.x > 0.5) mid = Math.max(mid, c); if (Hh.truck.state === S.Arrived) break; }
    wAll = Math.max(wAll, all); wMid = Math.max(wMid, mid); wX = Math.max(wX, Math.abs(cross(Hh.plant.pose, 0.8, 0.5, 0)));
  }
  return { wX, wAll, wMid };
}
function testLine() {
  // the brain's case: every drive starts where the previous step ended, i.e. on the leg line
  console.log('\nTEST line following (0.6 m leg, start on the line, 1 % slip, camera on, 20 runs)');
  const a = lineRuns(0.5, 20);
  console.log(`  worst TRUE cross-track during the drive ${f(a.wAll * 1000, 1)} mm, at the end ${f(a.wX * 1000, 1)} mm`);
  // PARITY with the twin, not an absolute: the same leg run 20 times in MATLAB with the twin's own model
  // (bot_step slip 1 %, cam_measure + pose_fusion, the brain_step drive law; 2026-11-19) gave worst 8.40 mm during
  // the drive and 4.69 mm at the end. The firmware law must not be worse than the twin it mirrors.
  check(a.wAll <= 0.0084, 'cross-track no worse than the twin in the same scenario (twin worst 8.4 mm)');
  // information only (no check): convergence from a 20 mm offset (not a brain case)
  const b = lineRuns(0.52, 20);
  console.log(`  info: start 20 mm off the line: worst cross-track over the second half ${f(b.wMid * 1000, 1)} mm, at the end ${f(b.wX * 1000, 1)} mm`);
}

function testTurn(cam) {
  const n = 60; console.log(`\nTEST turn in place (${n} runs, dir -1/0/+1, camera ${cam ? 'on' : 'off'})`);
  const r = rng(cam ? 22 : 21); const Ang = () => -kPi + kTwoPi * r.u();
  let ok = 0, dirOk = 0, worst = 0, wt = 0, wd = 0, mref = 0;
  for (let k = 0; k < n; k++) {
    const s = pose(0.5, 0.5, Ang()), h = Ang(), dir = (k % 3) - 1;
    const Hh = new Harness(200 + k, s); Hh.camOn = cam; Hh.run(0.2);
    const th0 = Hh.truck.est.th; Hh.send(turnCmd(1, h, dir));
    let swept = 0, prev = th0, arrived = false; const t0 = Hh.t;
    while (Hh.t - t0 < 10) { Hh.tick(); const th = Hh.truck.est.th; swept += wrapAngle(th - prev); prev = th;
      if (Hh.truck.state === S.Arrived && Hh.truck.seq === 1) { arrived = true; break; } }
    const e = Math.abs(rad2deg(wrapAngle(Hh.truck.est.th - h))), drift = dist(Hh.plant.pose, s.x, s.y);
    const eT = Math.abs(rad2deg(wrapAngle(Hh.plant.pose.th - h)));
    const good = arrived && (cam ? eT <= 4.5 : e <= 3); ok += good;
    const need = dir === 0 ? 0 : pyMod(dir * (h - th0), kTwoPi);
    const dg = dir === 0 || need < deg2rad(3) || need > kTwoPi - C.TURN_WRAP_SLACK || swept * dir > 0; dirOk += dg;
    worst = Math.max(worst, e); wt = Math.max(wt, eT); wd = Math.max(wd, drift); mref = Math.max(mref, Hh.maxRef);
    if (!good || !dg) console.log(`    run ${k}: dir ${dir} arrived ${arrived} err ${f(e)} deg swept ${f(swept)} rad`);
  }
  console.log(`  arrived within tolerance ${ok}/${n}, direction obeyed ${dirOk}/${n}, worst heading err ${f(worst)} deg (estimate) ${f(wt)} deg (true), worst centre drift ${f(wd * 1000, 1)} mm, max wheel ref ${f(mref, 3)} m/s`);
  check(ok === n, cam ? 'every turn ends with the TRUE heading within 4.5 deg, wheels stopped' : 'every turn ends within 3 deg (estimate) with the wheels stopped');
  check(dirOk === n, 'a forced direction (dir = +1 / -1) is obeyed');
}

function testSpeedLimit() {
  console.log('\nTEST wheel speed limit');
  const Hh = new Harness(301, pose(0.1, 0.1, 0)); const c = gotoCmd(1, 0.9, 0.9, kPi / 4); c.speed = 5;
  Hh.send(c); Hh.runUntilArrived(1, 30); Hh.send(turnCmd(2, 3.0, -1)); Hh.runUntilArrived(2, 10);
  const P = Params(); let wl = 0;
  for (let i = 0; i < 2000; i++) { const e = -kPi + kTwoPi * i / 2000;
    const a = gotoLaw(pose(0, 0, e), 1, 0, P.cruise, P, false).cmd, a2 = gotoLaw(pose(0, 0, e), 0.02, 0, P.cruise, P, false).cmd, b = turnLaw(0, e, 0, P.headTol, P).cmd;
    wl = Math.max(wl, Math.abs(a.vL), Math.abs(a.vR), Math.abs(a2.vL), Math.abs(a2.vR), Math.abs(b.vL), Math.abs(b.vR)); }
  console.log(`  max wheel speed command in a run ${f(Hh.maxCmd, 3)} m/s (reference ${f(Hh.maxRef, 3)}), max from the laws ${f(wl, 3)} m/s, limit ${C.MAX_WHEEL_SPEED}`);
  check(Hh.maxRef <= C.MAX_WHEEL_SPEED && Hh.maxCmd <= C.MAX_WHEEL_SPEED && wl <= C.MAX_WHEEL_SPEED, 'no wheel speed command exceeds 0.534 m/s');
}

function testWatchdog() {
  console.log('\nTEST watchdog (silence 300 ms)');
  const Hh = new Harness(401, pose(0.1, 0.5, 0)); Hh.camOn = false;
  Hh.send(gotoCmd(1, 0.9, 0.5, 0)); Hh.run(1.0);
  const vb = Hh.trueSpeed(); Hh.keepAlive = false; const ts = Hh.t; let tI = -1, tS = -1;
  while (Hh.t - ts < 2) { Hh.tick(); if (tI < 0 && Hh.truck.state === S.Idle) tI = Hh.t - ts; if (tI >= 0 && tS < 0 && Hh.trueSpeed() < 0.002) tS = Hh.t - ts; }
  const p1 = { ...Hh.plant.pose }; Hh.run(1.0); const creep = dist(Hh.plant.pose, p1.x, p1.y);
  console.log(`  speed before ${f(vb, 3)} m/s, idle ${f(tI, 3)} s after silence began (last repeat up to 0.1 s before), wheels < 2 mm/s ${f(tS, 3)} s after, creep while silent ${f(creep * 1000)} mm`);
  check(tI > 0 && tI <= 0.31, 'state idle within 300 ms of silence');
  check(tS > 0 && tS <= 0.31 + 0.2 / C.MAX_ACCEL + 0.1, 'wheels stopped within 300 ms + ramp-down');
  check(creep < 0.0005, 'robot stays put while silent');
  Hh.keepAlive = true; Hh.nextRepeat = Hh.t; Hh.tick(); const resumed = Hh.truck.state === S.Moving;
  const tt = Hh.runUntilArrived(1, 20);
  console.log(`  resumed ${resumed}, then arrived after ${f(tt)} s at ${f(dist(Hh.truck.est, 0.9, 0.5) * 1000, 1)} mm from the target`);
  check(resumed && tt > 0, 'keeps the goal and resumes when repeats return');
  const G = new Harness(402, pose(0.1, 0.5, 0)); G.keepAlive = false; G.send(gotoCmd(1, 0.9, 0.5, 0)); G.run(1.5);
  console.log(`  one-shot goto without repeats: travelled ${f(dist(G.plant.pose, 0.1, 0.5) * 1000, 1)} mm, state ${G.truck.state}`);
  check(dist(G.plant.pose, 0.1, 0.5) < 0.05 && G.truck.state === S.Idle, 'a command that is never repeated stops after 300 ms');
}

function testCamera() {
  console.log('\nTEST camera fusion');
  const Hh = new Harness(501, pose(0.2, 0.2, 0)); Hh.plant.wheelD[0] = C.WHEEL_DIAMETER_M * 1.01; Hh.plant.track = C.TRACK_M * 1.02; Hh.camOn = false; Hh.truck.setPose(pose(0.2, 0.2, 0));
  const pts = [[0.8, 0.2], [0.8, 0.8], [0.2, 0.8], [0.2, 0.2], [0.8, 0.2]]; let seq = 1, px = 0.2, py = 0.2;
  for (const p of pts) { Hh.send(gotoCmd(seq, p[0], p[1], Math.atan2(p[1] - py, p[0] - px))); Hh.runUntilArrived(seq, 30); Hh.send(turnCmd(seq + 100, Math.atan2(p[1] - py, p[0] - px) + kPi / 2, 1)); Hh.runUntilArrived(seq + 100, 10); seq++; px = p[0]; py = p[1]; }
  const err = () => dist(Hh.truck.est, Hh.plant.pose.x, Hh.plant.pose.y);
  const drift = err(), hd = Math.abs(rad2deg(wrapAngle(Hh.truck.est.th - Hh.plant.pose.th)));
  Hh.camOn = true; Hh.nextCam = Hh.t; Hh.run(1.0);
  const after = err(), ha = Math.abs(rad2deg(wrapAngle(Hh.truck.est.th - Hh.plant.pose.th)));
  console.log(`  encoders only after 2.4 m: estimate off by ${f(drift * 1000, 1)} mm / ${f(hd, 1)} deg; 1 s of camera fixes: ${f(after * 1000, 1)} mm / ${f(ha)} deg`);
  check(drift > 0.02, 'the drifted plant really drifts (>20 mm) without the camera');
  check(after < 0.005 && ha < 2, 'camera fixes pull the estimate back (<5 mm, <2 deg)');
  // latency compensation: driving at cruise, exact camera (no noise), image 50 ms old on arrival
  const lat = (ageReported) => { const P0 = Params(); if (!ageReported) P0.camDefaultLag = 0;   // no age, lag 0 = no compensation
    const L = new Harness(510, pose(0.1, 0.5, 0), P0); L.camNoise = false; L.ageReported = ageReported; L.camAge = 0.04;
    L.run(0.3); L.send(gotoCmd(1, 0.9, 0.5, 0)); let s = 0, n = 0, t0 = L.t;
    while (L.t - t0 < 4) { L.tick(); if (L.truck.state === S.Moving && L.trueSpeed() > 0.15) { s += Math.abs(L.plant.pose.x - L.truck.est.x); n++; } }
    return s / Math.max(n, 1); };
  const eComp = lat(true), eLag = lat(false);
  console.log(`  at cruise, mean along-track estimate error: with age ${f(eComp * 1000, 2)} mm, uncompensated (lag 0) ${f(eLag * 1000, 2)} mm`);
  check(eComp < 0.0015 && eLag > 2 * eComp, 'latency compensated: estimate does not lag the truth (< 1.5 mm at 0.2 m/s; uncompensated lags more)');
  const T = new Truck(); T.begin(pose(0.5, 0.5, 0), 0, 0, 1.0); T.setPose(pose(0.5, 0.5, 0));
  let now = 1.0; const tImg = 1000.0; for (let i = 0; i < 20; i++) { now += 0.005; T.step(now, 0, 0); }
  T.onCamera(0.53, 0.5, 0, tImg, 0.04, now); const e1 = 0.53 - T.est.x; T.onCamera(0.53, 0.5, 0, tImg + 0.033, 0.04, now + 0.033); const e2 = 0.53 - T.est.x;
  console.log(`  30 mm offset after fix 1: ${f(e1 * 1000, 1)} mm, after fix 2: ${f(e2 * 1000, 1)} mm`);
  check(Math.abs(e1 - 0.021) < 1e-4 && Math.abs(e2 - 0.0147) < 1e-4, 'complementary gain 0.3 per fix');
  T.setPose(pose(0.5, 0.5, deg2rad(179))); T.onCamera(0.5, 0.5, deg2rad(-179), tImg + 0.066, 0.04, now + 0.066);
  const hw = rad2deg(T.est.th); console.log(`  heading wrap: +179 fused with -179 -> ${f(hw)} deg`);
  check(Math.abs(Math.abs(hw) - 180) < 0.7, 'heading wrapped through +-180 (not via 0)');
  T.setPose(pose(0.5, 0.5, 0)); let r = T.onCamera(0.7, 0.5, 0, tImg + 0.1, 0.04, now + 0.1);
  check(r === CR.RejectedJump && Math.abs(T.est.x - 0.5) < 1e-6, 'a 200 mm jump is rejected');
  let k = 1, tt = 0.1; while (r !== CR.Relocked && k < 50) { tt += 0.033; r = T.onCamera(0.7, 0.5, 0, tImg + tt, 0.04, now + tt); k++; }
  console.log(`  relocked to the camera after ${k} consistent fixes`);
  check(r === CR.Relocked && k === C.CAM_RELOCK_COUNT && Math.abs(T.est.x - 0.7) < 1e-6, 'after 10 consistent fixes the camera wins');
  tt += 0.033; r = T.onCamera(0.7, 0.5, 0, tImg + tt - 0.5, 0.04, now + tt); check(r === CR.RejectedOrder, 'an out-of-order fix is rejected');
  r = T.onCamera(0.7, 0.5, 0, tImg + tt + 0.01, 0.30, now + tt + 0.40); check(r === CR.RejectedOld, 'a fix with age 0.30 s is rejected');
  r = T.onCamera(0.7, 0.5, 0, tImg + tt + 0.05, 0.04, now + tt + 0.05); check(r === CR.Accepted, 'the next timely fix is accepted again');
}

function testArcFloat32() {
  console.log('\nTEST odometry in float32 (ESP32 build): equal counts on both wheels, 200 ticks, distance estimate / truth');
  const R = Math.fround, P = Params();
  const mL = R(P.mPerCountL), mR = R(P.mPerCountR), tr = R(P.track);
  const run = (step, th0, n) => { let p = { x: 0, y: 0, th: R(th0) };
    for (let i = 0; i < 200; i++) { const sl = R(n * mL), sr = R(n * mR); p = step(p, R(0.5 * R(sl + sr)), R(R(sr - sl) / tr), R); }
    return Math.hypot(p.x, p.y) / (200 * n * 0.5 * (P.mPerCountL + P.mPerCountR)); };
  let worstNew = 0, worstOld = 0;
  for (const th of [Math.PI, 2.5, -3, 1.5708, 0.5, 0]) for (const n of [1, 2, 3, 4, 5, 8, 26]) {
    worstNew = Math.max(worstNew, Math.abs(run(arcStep, th, n) - 1)); worstOld = Math.max(worstOld, Math.abs(run(arcStepOld, th, n) - 1)); }
  console.log(`  worst |ratio - 1| over headings pi, 2.5, -3, 1.5708, 0.5, 0 and 1..26 counts/tick: chord form ${worstNew.toExponential(2)}, old form ${f(worstOld, 3)}`);
  check(worstNew < 1e-4, 'chord-form arc keeps the distance in float32 at every heading and speed (error < 1e-4)');
  check(worstOld > 0.5, 'the old difference-of-sines form fails in float32 (the test detects the bug)');
}

console.log(`motion_core JS mirror test: CONTROL_HZ ${C.CONTROL_HZ}, cruise ${C.CRUISE_SPEED} m/s, accel ${C.MAX_ACCEL} m/s^2, arrive ${C.ARRIVE_TOL_M * 1000} mm, turn tol ${C.HEADING_TOL_DEG} deg, fusion gain ${C.CAM_FUSION_GAIN}`);
testSemantics(); testGoto(true); testGoto(false); testLine(); testTurn(false); testTurn(true); testSpeedLimit(); testWatchdog(); testCamera(); testArcFloat32();
console.log(`\n${pass} passed, ${fail} failed`);
process.exit(fail === 0 ? 0 : 1);
