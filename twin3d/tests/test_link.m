% TEST_LINK  Phase 7 link to the real robots, up to (not including) real hardware. Prints "test_link PASSED".
%   Starts bridge.py + fake_bot.py in the background, ALWAYS in loopback mode (never on a real broker, where real
%   robots may be listening on the same topics), then
%   a. a goto command reaches a fake robot and telemetry with state "arrived" and the same seq comes back
%   b. command and telemetry payloads match the contract (link/TOPICS.md) field for field
%   c. a 30 s linked run: every robot's twin-vs-real error below 30 mm, zero safety violations in the twin,
%      commands acknowledged, and the same again with syncToReal (20 s)
%   d. the watchdog: MATLAB silent (no heartbeat) -> the bridge stops repeating after P.link.hostTimeout -> the
%      robot's watchdog (P.link.watchdog) stops it; heartbeats again -> it resumes and arrives
%   f. keep-alive across a brain stall: MATLAB sends NOTHING for 0.7 s (longer than the 300 ms watchdog, shorter
%      than hostTimeout) during a drive; the bridge's repeats keep the robot moving (never idle)
%   e. every process started is gone afterwards (checked by PID)
%   If Python is missing the test prints SKIP and passes.
here = fileparts(mfilename('fullpath'));
P = twin_params();
[exe, hasPaho, pyVer] = link_python();
if isempty(exe)
    fprintf('  SKIP  test_link: Python is not available (python on the PATH); the link cannot be tested\n');
    return
end
plan = load_plan(fullfile(here, '..', 'plan', 'plan_arena.json'));
fprintf('  Python %s, paho-mqtt %s\n', pyVer, mat2str(hasPaho));

procs = link_launch(P, struct('mode', 'loopback'));
cleaner = onCleanup(@() link_kill(procs));
fprintf('  bridge mode: %s (port %d), PIDs %s\n', procs.mode, procs.port, mat2str(procs.pids));
assert(all(procs.pids > 0) && numel(procs.pids) >= 2, 'processes not started');
assert(strcmp(procs.mode, 'loopback'), 'the test bridge must be in loopback mode, got %s', procs.mode);
L = link_open(P, struct('port', procs.port));
guard = onCleanup(@() link_close(L, false));
% command counters start at a time-based base, not at 1: a real robot keeps the seq of the last session's
% last command, and a new command with that same seq would be taken for a keep-alive (TOPICS.md)
s0 = L.st('seq'); assert(all(s0 > 1000), 'seq base %s', mat2str(s0));

% ---- a. goto -> arrived (robot 1 drives 0.12 m west along the queue row)
slot = P.brain.parking;
cmd = struct('id', 1, 'truck', 1, 'action', 'goto', 'x', slot(1,1) - 0.12, 'y', slot(1,2), 'heading', pi, 'speed', 0.2);
cmd = link_send_cmd(L, cmd);
tel = []; t0 = tic;
while toc(t0) < 8
    link_keepalive(L, 0.1);
    R = link_poll(L);
    for q = 1:numel(R.telemetry)
        r = R.telemetry(q);
        if r.id == 1 && r.seq == cmd.seq && strcmp(r.state, 'arrived'), tel = r; end
    end
    if ~isempty(tel), break; end
    pause(0.02);
end
assert(~isempty(tel), 'a: no "arrived" telemetry within 8 s');
errA = hypot(tel.x - cmd.x, tel.y - cmd.y);
assert(errA < 0.015, 'a: robot reports %.1f mm from the goal', 1000*errA);
fprintf('  a. goto reached the fake robot, arrived after %.2f s, reported position %.1f mm from the goal\n', toc(t0), 1000*errA);
pause(0.1); R = link_poll(L); %#ok<NASGU>

% ---- b. payloads field for field (raw lines straight from the bridge)
% cmd as sent by MATLAB: re-encode through the same function and compare the field set
cmdFields = {'id', 'truck', 'seq', 'action', 'x', 'y', 'heading', 'speed'};
sent = struct('id', 2, 'truck', 5, 'action', 'turn', 'x', 0.5, 'y', 0.5, 'heading', 1, 'speed', 0.2, 'dir', -1);
% a second client on the bridge plays the broker side: it subscribes to the cmd topic and reads what MATLAB published
spy = tcpclient('127.0.0.1', procs.port, 'Timeout', 1);
write(spy, uint8([jsonencode(struct('subscribe', {{'zadp/+/cmd', 'zadp/+/telemetry', 'zadp/cam/pose'}})) 10]));
pause(0.3);
link_send_cmd(L, struct('id', 3, 'truck', 4, 'action', 'goto', 'x', 0.8, 'y', -0.159, 'heading', pi, 'speed', 0.2));
link_send_cmd(L, sent);
pause(0.7);
raw = char(read(spy, spy.NumBytesAvailable, 'uint8'));
delete(spy);
lines = strsplit(strtrim(raw), newline);
seen = struct('cmd', 0, 'tel', 0, 'cam', 0);
for i = 1:numel(lines)
    m = jsondecode(lines{i});
    if endsWith(m.topic, '/cmd')
        want = cmdFields; if strcmp(m.payload.action, 'turn'), want = [want {'dir', 'tol'}]; end
        assert(isequal(sort(fieldnames(m.payload)'), sort(want)), 'b: cmd fields %s', strjoin(fieldnames(m.payload), ','));
        assert(strcmp(m.topic, sprintf('zadp/bot%d/cmd', m.payload.id)), 'b: cmd topic');
        assert(ismember(m.payload.action, {'goto', 'turn', 'dump', 'stop'}) && m.payload.seq >= 1, 'b: cmd values');
        seen.cmd = seen.cmd + 1;
    elseif endsWith(m.topic, '/telemetry')
        want = {'id', 'seq', 'encL', 'encR', 'x', 'y', 'heading', 'state'};
        assert(isequal(sort(fieldnames(m.payload)'), sort(want)), 'b: telemetry fields %s', strjoin(fieldnames(m.payload), ','));
        assert(strcmp(m.topic, sprintf('zadp/bot%d/telemetry', m.payload.id)), 'b: telemetry topic');
        assert(ismember(m.payload.state, {'idle', 'moving', 'turning', 'dumping', 'arrived'}), 'b: state %s', m.payload.state);
        assert(m.payload.encL == round(m.payload.encL) && m.payload.encR == round(m.payload.encR), 'b: counts not integer');
        seen.tel = seen.tel + 1;
    elseif strcmp(m.topic, 'zadp/cam/pose')
        assert(isequal(sort(fieldnames(m.payload)'), sort({'bot', 'x', 'y', 'heading', 't', 'age'})), 'b: cam fields');
        assert(m.payload.age >= 0 && m.payload.age < P.cam.maxAge, 'b: cam age %g', m.payload.age);
        assert(abs(m.payload.heading) <= pi + 1e-6 && ismember(m.payload.bot, 1:3), 'b: cam values');
        seen.cam = seen.cam + 1;
    end
end
assert(seen.cmd >= 2 && seen.tel > 20 && seen.cam > 20, 'b: saw %d cmd, %d telemetry, %d cam', seen.cmd, seen.tel, seen.cam);
fprintf('  b. payloads match the contract: %d cmd, %d telemetry, %d cam messages checked field for field\n', seen.cmd, seen.tel, seen.cam);
% leave the robots as they were
link_stop_all(L); pause(0.3);

% ---- d. watchdog: a long goto, then no more command lines
link_close(L, true); delete(guard); clear L
left1 = link_kill(procs); assert(isempty(left1), 'processes left: %s', mat2str(left1));
procs2 = link_launch(P, struct('mode', 'loopback'));         % fresh robots at the parking slots
cleaner2 = onCleanup(@() link_kill(procs2));
L = link_open(P, struct('port', procs2.port));
guard2 = onCleanup(@() link_close(L, false));
% far goal (0.74 m west): the robot is still driving when the bridge stops repeating hostTimeout after the silence starts
cmd = link_send_cmd(L, struct('id', 1, 'truck', 1, 'action', 'goto', 'x', slot(1,1) - 0.74, 'y', slot(1,2), 'heading', pi, 'speed', 0.2));
t0 = tic; last = [];
while toc(t0) < 0.6                                       % keep-alives for 0.6 s: the robot moves
    link_keepalive(L, 0.1); R = link_poll(L); last = pickLast(R, 1, last); pause(0.02);
end
assert(~isempty(last) && strcmp(last.state, 'moving'), 'd: robot not moving');
xMove = last.x;
% silence: no heartbeat, no command. The bridge repeats for hostTimeout, then the watchdog stops the robot
tSil = P.link.hostTimeout + P.link.watchdog + 0.7;
t1 = tic; while toc(t1) < tSil, R = link_poll(L); last = pickLast(R, 1, last); pause(0.02); end
x1 = last.x; assert(strcmp(last.state, 'idle'), 'd: state %s after %.1f s of silence', last.state, tSil);
t1 = tic; while toc(t1) < 0.5, R = link_poll(L); last = pickLast(R, 1, last); pause(0.02); end
assert(abs(last.x - x1) < 0.010, 'd: robot still moving 1 s after the last command line (moved %.1f mm; the estimate itself jitters ~3 mm with the camera fix)', 1000*abs(last.x - x1));
fprintf('  d. watchdog: MATLAB silent %.1f s -> repeats stopped after hostTimeout %.1f s -> robot idle (x %.3f at the last heartbeat, %.3f stopped)\n', tSil, P.link.hostTimeout, xMove, x1);
t1 = tic; while toc(t1) < 6                               % keep-alives again: it resumes and arrives
    link_keepalive(L, 0.1); R = link_poll(L); last = pickLast(R, 1, last); pause(0.02);
    if strcmp(last.state, 'arrived'), break; end
end
assert(strcmp(last.state, 'arrived'), 'd: robot did not resume after the keep-alives came back');

% ---- f. a brain stall (no line from MATLAB for 0.7 s > watchdog) does not stop a driving robot
cmd = link_send_cmd(L, struct('id', 1, 'truck', 1, 'action', 'goto', 'x', last.x - 0.25, 'y', slot(1,2), 'heading', pi, 'speed', 0.2));
t1 = tic; while toc(t1) < 0.5, link_keepalive(L); R = link_poll(L); last = pickLast(R, 1, last); pause(0.02); end
assert(strcmp(last.state, 'moving'), 'f: robot not moving before the stall (%s)', last.state);
pause(0.7);                                              % the stall: nothing sent, nothing read
R = link_poll(L); st = {R.telemetry([R.telemetry.id] == 1).state};
assert(numel(st) >= 10 && all(strcmp(st, 'moving') | strcmp(st, 'arrived')), 'f: robot stopped during a 0.7 s stall: %s', strjoin(unique(st), ','));
fprintf('  f. 0.7 s stall without any MATLAB line: %d telemetry messages during it, all moving (bridge repeats at %g Hz)\n', numel(st), P.link.repeatHz);
link_close(L, true); delete(guard2); clear L
left2 = link_kill(procs2); assert(isempty(left2), 'd: processes left: %s', mat2str(left2));

% ---- c. linked runs against fresh fake robots (the twin leads)
procs3 = link_launch(P, struct('mode', 'loopback'));
cleaner3 = onCleanup(@() link_kill(procs3));
L = link_open(P, struct('port', procs3.port));
guard3 = onCleanup(@() link_close(L));
for sync = [false true]
    maxT = 30 - 10*sync;
    [~, log, lk] = brain_run_linked(plan, P, struct('link', L, 'maxTime', maxT, 'syncToReal', sync));
    m = log.mon;
    assert(m.rrOverlapSteps == 0 && m.pileHitSteps == 0 && m.boundSteps == 0 && m.deadlocks == 0, ...
        'c: safety violations in the twin (rr %d, pile %d, bound %d, deadlock %d)', m.rrOverlapSteps, m.pileHitSteps, m.boundSteps, m.deadlocks);
    assert(all(lk.worstErrBot < 0.030), 'c: twin-vs-real error %s mm (sync %d)', mat2str(round(1000*lk.worstErrBot, 1)), sync);
    % the last command may be sent in the final step and acknowledged after the run's 50 ms drain
    assert(lk.nCmd >= 5 && numel(lk.latencyMs) >= lk.nCmd - 1, 'c: %d commands, %d acknowledged', lk.nCmd, numel(lk.latencyMs));
    assert(m.ownerSteps == 0, 'c: section ownership violated (%d steps)', m.ownerSteps);
    assert(lk.nTel > 10*maxT && lk.nCam > 10*maxT, 'c: too few messages (%d telemetry, %d cam)', lk.nTel, lk.nCam);
    assert(lk.holdTimeouts == 0, 'c: %d hold timeouts', lk.holdTimeouts);
    fprintf('  c. linked run %2.0f s, syncToReal %d: %d commands, %d telemetry, %d cam; error mean %.1f mm, worst %.1f mm; latency mean %.1f ms; twin violations 0; hold %s s\n', ...
        maxT, sync, lk.nCmd, lk.nTel, lk.nCam, 1000*lk.meanErr, 1000*lk.worstErr, lk.meanLatencyMs, mat2str(round(lk.holdTime, 2)));
    pause(0.3);
    if ~sync    % fresh robots for the second run: they must start at the parking slots like the new twin
        link_close(L, true); delete(guard3); left3 = link_kill(procs3); assert(isempty(left3), 'c: processes left');
        procs3 = link_launch(P, struct('mode', 'loopback'));
        cleaner3 = onCleanup(@() link_kill(procs3));
        L = link_open(P, struct('port', procs3.port));
        guard3 = onCleanup(@() link_close(L));
    end
end

% ---- e. no process left (by PID)
link_close(L, true); delete(guard3);
allPids = unique([procs.pids, procs2.pids, procs3.pids]);
link_kill(procs); link_kill(procs3);
left = link_kill(struct('pids', allPids));
assert(isempty(left), 'e: processes still running: %s', mat2str(left));
fprintf('  e. all %d started processes are gone (PIDs %s)\n', numel(allPids), mat2str(allPids));
fprintf('test_link PASSED (mode %s)\n', procs.mode);

function last = pickLast(R, id, last)
for q = 1:numel(R.telemetry)
    if R.telemetry(q).id == id, last = R.telemetry(q); end
end
end
