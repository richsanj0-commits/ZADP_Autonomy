% TEST_BOT  Phase 3 single-robot physics, odometry, camera fusion, go-to-point. Prints "test_bot PASSED".
rng(1);
P = twin_params();
dt = P.sim.dt;

% a. one wheel revolution -> 2772 +-1 counts
s = bot_init(P, [0.5 0.5 0], 1);
while s.wheelAngle(1) < 2*pi
    s = bot_step(s, [0.2 0.2], P, 1e-4);   % fine step so the stop lands on the revolution
end
assert(abs(s.enc(1) - 2772) <= 1, 'counts per rev %d', s.enc(1));
fprintf('  a. counts for one revolution: %d\n', s.enc(1));

% b. slipStd = 0, no camera, 0.5 m straight
Pb = P; Pb.bot.slipStd = 0; Pb.ctrl.arriveTol = 0.001;
s = bot_init(Pb, [0.2 0.5 0], 1);
[s, lg] = bot_goto(s, [0.7 0.5], Pb, struct('useCamera', false));
assert(lg.arrived, 'b: not arrived');
errT = hypot(s.pose(1)-0.7, s.pose(2)-0.5);
errO = hypot(s.pose(1)-s.odom(1), s.pose(2)-s.odom(2));
assert(errT < 0.002, 'b: true error %.2f mm', 1000*errT);
assert(errO < 0.0005, 'b: odometry vs truth %.3f mm', 1000*errO);
fprintf('  b. straight 0.5 m: true error %.2f mm, odom-vs-truth %.3f mm\n', 1000*errT, 1000*errO);

% c. rotate to a target behind the robot, then reach it
s = bot_init(P, [0.5 0.5 0], 1);
[s, lg] = bot_goto(s, [0.3 0.5], P, struct('useCamera', true));
assert(lg.arrived, 'c: not arrived');
assert(max(abs(lg.v(:))) <= P.bot.maxWheelSpeed + 1e-9, 'c: wheel speed above max');
acc = diff([0 0; lg.v], 1, 1)/dt;
assert(max(abs(acc(:))) <= P.bot.maxAccel + 1e-6, 'c: accel %.4f above max', max(abs(acc(:))));
fprintf('  c. turn-around: arrived in %.2f s, max |v| %.3f, max accel %.3f\n', lg.tArrive, max(abs(lg.v(:))), max(abs(acc(:))));

% d/e. ten trips 0.6 m out and back, slipStd = 0.01
A = [0.2 0.5]; B = [0.8 0.5];
for useCam = [false true]
    s = bot_init(P, [A 0], 1); errs = zeros(1,20); ok = true; dist = 0;
    for k = 1:10
        for tgt = [B; A]'
            [s, lg] = bot_goto(s, tgt', P, struct('useCamera', useCam));
            ok = ok && lg.arrived;
            dist = dist + 0.6;
            errs(1 + (k-1)*2 + double(all(tgt' == A))) = hypot(s.pose(1)-tgt(1), s.pose(2)-tgt(2));
        end
    end
    assert(ok, 'd/e: a leg did not arrive');
    if ~useCam
        endErr = errs(end);
        assert(endErr > 0.003, 'd: odometry-only error only %.2f mm', 1000*endErr);
        fprintf('  d. camera OFF, 10 round trips (%.1f m): true error after trip 1 %.1f mm, after trip 10 %.1f mm; odometry believes %.1f mm\n', ...
            dist, 1000*errs(2), 1000*endErr, 1000*hypot(s.odom(1)-A(1), s.odom(2)-A(2)));
    else
        assert(max(errs) < 0.010, 'e: worst arrival error %.2f mm', 1000*max(errs));
        fprintf('  e. camera ON, 10 round trips: worst arrival error %.2f mm\n', 1000*max(errs));
    end
end

% f. demo path arrives and stays inside the arena
res = demo_one_bot('', false);
assert(res.arrived, 'f: demo did not arrive');
assert(res.inArena, 'f: robot left the arena');
fprintf('  f. demo: trip %.1f s, final error %.1f mm\n', res.tTrip, 1000*res.finalError);

% g. camera latency: the fix shows the pose of P.cam.latency ago. Straight drive at cruise speed, 300
%    steady-state steps. Fused uncompensated, the estimate lags by about v*latency (10 mm); with the
%    odometry compensation in pose_fusion the along-track bias must be under 2 mm.
rs = RandStream('mt19937ar', 'Seed', 5); bias = zeros(1, 2);
for comp = [false true]
    s = bot_init(P, [0.1 0.5 0], 1, rs); nc = 0; e = zeros(0, 1);
    for k = 1:600
        s = bot_step(s, [1 1]*P.bot.cruiseSpeed, P, dt);
        if s.t >= nc - 1e-12
            [z, tz] = cam_measure(s, P);
            if comp, s = pose_fusion(s, z, P, tz); else, s = pose_fusion(s, z, P); end
            nc = nc + 1/P.cam.rateHz;
        end
        if k > 300, e(end+1, 1) = s.est(1) - s.pose(1); end %#ok<SAGROW>
    end
    bias(comp + 1) = mean(e);
end
assert(abs(bias(1)) > 0.5*P.bot.cruiseSpeed*P.cam.latency, 'g: latency not modelled (bias %.2f mm)', 1000*bias(1));
assert(abs(bias(2)) < 0.002, 'g: compensated along-track bias %.2f mm', 1000*bias(2));
fprintf('  g. camera latency %.0f ms: along-track bias %.1f mm uncompensated, %.2f mm compensated\n', 1000*P.cam.latency, 1000*bias(1), 1000*bias(2));

% h. wheel travel is dt-independent while accelerating (trapezoidal step): 1 s from rest, no slip
Pz = P; Pz.bot.slipStd = 0; dist = zeros(1, 3); dts = [0.005 0.01 0.02];
for q = 1:3
    s = bot_init(Pz, [0 0 0], 1, rs);
    for k = 1:round(1/dts(q)), s = bot_step(s, [1 1]*P.bot.cruiseSpeed, Pz, dts(q)); end
    dist(q) = s.pose(1);
end
assert(max(dist) - min(dist) < 1e-4, 'h: distance depends on dt by %.3f mm', 1000*(max(dist) - min(dist)));
fprintf('  h. 1 s from rest: %.5f / %.5f / %.5f m at dt %s\n', dist, mat2str(dts));

% i. near the target the controller never rotates in place (capture radius) and does not chatter
%    between drive and rotate; 40 random go-to runs with the camera on, brain step dt
rs = RandStream('mt19937ar', 'Seed', 7); errs = zeros(1, 40); maxSw = 0; rotNear = 0; dti = 0.02;
for k = 1:40
    p0 = [0.3 + 0.4*rand(rs, 1, 2), 2*pi*rand(rs)]; tg = 0.3 + 0.4*rand(rs, 1, 2);
    s = bot_init(P, p0, 1, rs); nc = 0; rot = false; prev = false; sw = 0; arr = false;
    for it = 1:round(30/dti)
        [cmd, arr, rot] = bot_controller(s.est, tg, P, rot);
        if rot ~= prev, sw = sw + 1; end, prev = rot;
        if rot && hypot(tg(1) - s.est(1), tg(2) - s.est(2)) < P.ctrl.captureRadius, rotNear = rotNear + 1; end
        if arr, break, end
        s = bot_step(s, cmd, P, dti);
        if s.t >= nc - 1e-12, [z, tz] = cam_measure(s, P); s = pose_fusion(s, z, P, tz); nc = nc + 1/P.cam.rateHz; end
    end
    assert(arr, 'i: run %d did not arrive', k);
    for j = 1:100, if all(abs(s.v) < 1e-4), break, end, s = bot_step(s, [0 0], P, dti); end
    errs(k) = hypot(tg(1) - s.pose(1), tg(2) - s.pose(2)); maxSw = max(maxSw, sw);
end
assert(rotNear == 0, 'i: rotated in place inside the capture radius (%d steps)', rotNear);
assert(maxSw <= 4, 'i: %d drive/rotate switches in one run', maxSw);
assert(max(errs) < 0.010, 'i: true arrival error %.2f mm', 1000*max(errs));
fprintf('  i. 40 random go-tos: true error mean %.2f max %.2f mm, max drive/rotate switches %d\n', 1000*mean(errs), 1000*max(errs), maxSw);

disp('test_bot PASSED');
