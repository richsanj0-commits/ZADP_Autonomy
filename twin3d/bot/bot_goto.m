function [s, log] = bot_goto(s, target, P, opts)
%BOT_GOTO  Run controller + bot_step (+ camera fixes + pose_fusion) until arrived or timeout.
%   opts.useCamera (default true), opts.timeout (60 s), opts.dt (P.sim.dt).
%   Controller uses s.est when the camera is on, s.odom when off. After arrival the robot is braked
%   to a stop (cmd 0) before returning.
%   log: t, truePose, odom, est (N-by-3), cmd, v, enc (N-by-2), arrived, tArrive.
if nargin < 4, opts = struct(); end
useCam = ~isfield(opts,'useCamera') || opts.useCamera;
timeout = 60; if isfield(opts,'timeout'), timeout = opts.timeout; end
dt = P.sim.dt; if isfield(opts,'dt'), dt = opts.dt; end
n = ceil(timeout/dt) + ceil(2/dt) + 2;
L.t = zeros(n,1); L.truePose = zeros(n,3); L.odom = zeros(n,3); L.est = zeros(n,3);
L.cmd = zeros(n,2); L.v = zeros(n,2); L.enc = zeros(n,2);
k = 0; arrived = false; tArrive = NaN; t0 = s.t; nextCam = s.t; rot = false;
while true
    if useCam, p = s.est; else, p = s.odom; end
    [cmd, arr, rot] = bot_controller(p, target, P, rot);
    if arr && ~arrived, arrived = true; tArrive = s.t - t0; tBrake = s.t; end
    if arrived
        cmd = [0 0];
        if all(abs(s.v) < 1e-4) || s.t - tBrake > 2, break; end
    elseif s.t - t0 >= timeout
        break
    end
    s = bot_step(s, cmd, P, dt);
    if useCam && s.t >= nextCam - 1e-12
        [z, tz] = cam_measure(s, P);
        s = pose_fusion(s, z, P, tz);
        nextCam = nextCam + 1/P.cam.rateHz;
    end
    k = k + 1;
    L.t(k) = s.t; L.truePose(k,:) = s.pose; L.odom(k,:) = s.odom; L.est(k,:) = s.est;
    L.cmd(k,:) = cmd; L.v(k,:) = s.v; L.enc(k,:) = s.enc;
end
log = structfun(@(a) a(1:k,:), L, 'UniformOutput', false);
log.arrived = arrived; log.tArrive = tArrive;
end
