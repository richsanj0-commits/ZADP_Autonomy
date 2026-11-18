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

disp('test_bot PASSED');
