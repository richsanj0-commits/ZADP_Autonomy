function R = run_real(mode, opts)
%RUN_REAL  One command to run the twin with the REAL robots (the twin leads, the robots follow).
%   R = run_real()            real hardware: broker, 3 ESP32 robots, webcam + ArUco markers
%   R = run_real('real', struct('realBots', 1))   robot 1 real, robots 2 and 3 simulated by fake_bot.py on the same
%                             broker (the 3-robot twin runs with 1, 2 or 3 real robots)
%   R = run_real('dry')       the SAME code path without hardware: loopback bridge, fake_bot.py robots (no camera of
%                             their own) and aruco_node.py --sim (synthetic marker pixels through the real geometry)
%   R = run_real('bench', struct('realBots', [2 3]))   BENCH: robots 2 and 3 are bare ESP32 boards running the firmware
%                             bench build (-DBENCH_SIM=1, motors and encoders simulated on the chip), the others are
%                             simulated by fake_bot.py as in the mixed mode. No camera and no OpenCV: bench_cam.py
%                             republishes each bench robot's own telemetry pose as zadp/cam/pose, after first putting the
%                             robot on its parking slot (a fake camera fix; see py/bench_cam.py). Never with robots on
%                             the floor. 'real' and 'dry' are unchanged.
%   R = run_real(mode, opts)  opts: maxTime (sim s = wall s, default 600 real / 30 dry), syncToReal (true),
%                             show (live 3D view; default true real / false dry: the view is then built hidden and
%                             still driven, so the path is tested), viewDt (s between view refreshes, 0.2),
%                             camArgs (extra aruco_node.py arguments, e.g. '--camera 1 --width 1920 --height 1080'),
%                             plan (plan file, default plan/plan_arena.json), seed (1),
%                             realBots (ids of the real robots, default 1:P.bot.count; the others are simulated by
%                             fake_bot.py with their own camera pose; in dry the real ones are fake_bot.py stand-ins
%                             seen by the simulated camera),
%                             dry only: init (N-by-3 fake robot start poses, row i = robot i, default the parking slots),
%                             simHideCorner (corner ids hidden from the simulated camera, e.g. '11'),
%                             bench only: benchUnflagged (true = accept boards whose retained status has no "bench":true,
%                             flashed before the flag existed; default false: bench_cam.py refuses them),
%                             viewHook (function viewHook(W, fig) called after each view refresh; the test uses it
%                             to close the view like a user)
%   Steps (each prints one line; any failure stops with a message that says what to fix):
%     1. writes link/link_config.json from twin_params (the single source; every helper reads it)
%     2. preflight: Python, numpy, paho-mqtt (real), OpenCV with cv2.aruco (real), broker reachable at
%        P.mqtt.broker:P.mqtt.port (real; when none answers and that host is this PC, link_launch starts
%        py/mini_broker.py and link_kill stops it at the end)
%     3. starts bridge.py (real / bench: broker mode, no fallback; dry: loopback), aruco_node.py (publishes the real
%        robots only; bench: bench_cam.py instead), fake_bot.py for the simulated robots (+ stand-ins for the real
%        ones in dry); waits until the camera node has seen all corner markers (P.link.camStartTimeout); bench: until
%        bench_cam.py has put every bench robot on its parking slot and says "bench ready" (error text in its log)
%     4. waits for telemetry AND a camera pose from every robot (P.link.readyTimeout)
%     5. checks every REAL robot i stands on parking slot P<i> (P.link.parkTol, P.link.parkTolDeg) and names any that
%        does not (a simulated robot starts on its slot by construction)
%     6. brain_run_linked with syncToReal and the live 3D view drawn at the REAL (camera) poses
%   Any error, Ctrl+C or closing the view sends stop to every robot, closes the link and kills every helper process.
%   The run ends early (one line "ended early ..." with the reason, then the error) when a robot goes offline or silent,
%   the camera loses a robot for P.link.camLossTime (link:camLost), a robot stays more than P.link.divergeLimit from its
%   twin for P.link.divergeTime (link:diverged), or the camera helper process exits or logs an error (link:aborted;
%   every mode, checked every P.link.camCheckDt).
%   R: mode, realBots, simBots, lk, mon (twin safety monitors), log, procs, checks (preflight / corners (bench: bench) / ready flags,
%   park = bot, dist, headingDeg per real robot), camLog.
if nargin < 1 || isempty(mode), mode = 'real'; end
if nargin < 2, opts = struct(); end
mode = validatestring(mode, {'real', 'dry', 'bench'});
dry = strcmp(mode, 'dry');
bench = strcmp(mode, 'bench');
here = fileparts(mfilename('fullpath'));
def = struct('maxTime', 600 - 570*dry, 'syncToReal', true, 'show', ~dry, 'viewDt', 0.2, 'camArgs', '', ...
    'plan', fullfile(fileparts(here), 'plan', 'plan_arena.json'), 'seed', 1, 'init', [], 'simHideCorner', '', 'viewHook', [], ...
    'realBots', 'all', 'benchUnflagged', false);
f = fieldnames(def);
for i = 1:numel(f), if ~isfield(opts, f{i}), opts.(f{i}) = def.(f{i}); end, end

P = twin_params();
nb = P.bot.count;
% rule: bench mode needs the bench robots named explicitly: run_real('bench') alone must never address robot 1, the real
% floor robot (a fake camera would be fed to it)
assert(~(bench && ischar(opts.realBots)), 'run_real:realBots', ...
    'bench mode needs opts.realBots, e.g. struct(''realBots'', [2 3]): only bare bench boards, never a robot on the floor');
if ischar(opts.realBots), opts.realBots = 1:nb; end
realBots = unique(opts.realBots(:)');
assert(all(ismember(realBots, 1:nb)), 'run_real:realBots', 'realBots must be robot ids 1..%d', nb);
assert(~bench || ~isempty(realBots), 'run_real:realBots', 'bench mode needs at least one bench robot in realBots');
simBots = setdiff(1:nb, realBots);
plan = load_plan(opts.plan);
Lay = brain_layout(plan, P);
R = struct('mode', mode, 'realBots', realBots, 'simBots', simBots, 'lk', [], 'mon', [], 'log', [], 'procs', [], ...
    'checks', struct(), 'camLog', '');
say = @(varargin) fprintf(['run_real(%s): ' varargin{1} '\n'], mode, varargin{2:end});

% ---- 1. config (single source)
cfgFile = link_write_config(P);
say('numbers from twin_params written to %s', cfgFile);

% ---- 2. preflight
[exe, hasPaho, pyVer] = link_python();
if isempty(exe), error('run_real:python', 'Python not found on the PATH (install Python 3 and make "python" work in a shell).'); end
say('[ok] Python %s (%s)', pyVer, exe);
need = {'numpy', 'pip install numpy'};
% rule: OpenCV only for the real camera (dry: synthetic pixels, bench: telemetry echo)
if ~dry && ~bench, need(end+1, :) = {'cv2; assert hasattr(cv2, "aruco")', 'pip install opencv-contrib-python'}; end
for k = 1:size(need, 1)
    [st, out] = system(sprintf('"%s" -c "import %s"', exe, need{k, 1}));
    if st ~= 0
        error('run_real:pydeps', 'Python module missing: import %s failed (%s). Install it yourself: %s', ...
            need{k, 1}, strtrim(out), need{k, 2});
    end
    say('[ok] Python module %s', strtok(need{k, 1}, ';'));
end
if ~dry
    if ~hasPaho, error('run_real:pydeps', 'paho-mqtt missing: pip install paho-mqtt'); end
    say('[ok] Python module paho.mqtt');
    [up, isLocal] = link_broker_status(P);
    if up
        say('[ok] broker reachable at %s:%d (used as it is)', P.mqtt.broker, P.mqtt.port);
    elseif isLocal
        % rule: no broker on this PC -> link_launch starts py/mini_broker.py on 0.0.0.0:P.mqtt.port (killed at the end)
        say('[ok] no broker at %s:%d: link_launch starts mini_broker.py on this PC (port %d, all interfaces)', ...
            P.mqtt.broker, P.mqtt.port, P.mqtt.port);
    else
        error('run_real:broker', ['no MQTT broker at %s:%d, and that host is not this PC (mini_broker.py runs only ' ...
            'here). Start a broker there (Mosquitto with "listener %d 0.0.0.0" and "allow_anonymous true", or ' ...
            'python link\\py\\mini_broker.py), allow TCP %d in its firewall, or set P.mqtt.broker in twin_params.m ' ...
            'to this PC (the firmware secrets.h MQTT_HOST must point at the same PC).'], P.mqtt.broker, P.mqtt.port, ...
            P.mqtt.port, P.mqtt.port);
    end
end
R.checks.preflight = true;

% ---- 3. helpers
% rule: the camera node publishes the real robots only; the simulated ones publish their own camera pose
lo = struct('mode', 'loopback', 'bots', realBots, 'fakeCam', false, 'simBots', simBots, 'realBots', [], ...
    'camNode', 'sim', 'camBots', realBots, 'camArgs', opts.camArgs, 'config', cfgFile, 'ttl', opts.maxTime + 600, ...
    'port', P.link.bridgePort, 'init', Lay.slot);
if dry
    % dry: fake_bot.py stand-ins (no camera of their own, --truth) play the real robots
    lo.port = [];                                         % random port: never collides with a real bridge
    if ~isempty(opts.simHideCorner), lo.camArgs = strtrim([lo.camArgs ' --sim-hide-corner ' opts.simHideCorner]); end
    if ~isempty(opts.init), lo.init = opts.init; end
else
    lo.mode = 'broker'; lo.camNode = 'real'; lo.bots = []; lo.realBots = realBots;
    % rule: bench boards have no camera: bench_cam.py (broker, own telemetry as camera pose, start pose = parking slot)
    if bench, lo.camNode = 'bench'; end
    if bench && opts.benchUnflagged, lo.camArgs = strtrim([lo.camArgs ' --allow-unflagged']); end
end
say('real robots %s, simulated robots %s', mat2str(realBots), mat2str(simBots));
h = [];
% cleanup state in a handle object: an onCleanup must not read the function's own variables (MATLAB may have
% destroyed them already when an error unwinds), so it gets this map, filled as things start
ctx = containers.Map();
cleanup = onCleanup(@() shutdownReal(ctx));
procs = link_launch(P, lo);
ctx('procs') = procs;
R.procs = procs;
say('[ok] bridge mode %s on port %d, helpers %s (logs in %s)', procs.mode, procs.port, mat2str(procs.pids), tempdir);
if strcmp(procs.broker, 'mini')
    say('[ok] mini_broker.py started on port %d (log %s); robots must reach this PC on TCP %d (firewall: allow Python on private networks)', ...
        P.mqtt.port, procs.logs.broker, P.mqtt.port);
end
if ~dry && ~strcmp(procs.mode, 'broker'), error('run_real:bridge', 'bridge is not in broker mode (%s)', procs.mode); end
t0 = tic;
camWord = 'homography ready';
if bench, camWord = 'bench ready'; end
while ~contains(readLog(procs.logs.cam), camWord)
    % rule: a failed bench_cam.py says ERROR (it logs any exception that way) or died with a traceback
    if bench && (contains(readLog(procs.logs.cam), 'ERROR') || contains(readLog(procs.logs.cam), 'Traceback'))
        error('run_real:bench', 'bench_cam.py stopped (status not a bench build, no telemetry, slot not reached, ...):\n%s', ...
            lastLines(readLog(procs.logs.cam), 6));
    end
    if bench && toc(t0) > P.link.camStartTimeout + P.link.readyTimeout
        error('run_real:bench', 'bench_cam.py not ready within %g s. Last log lines:\n%s', ...
            P.link.camStartTimeout + P.link.readyTimeout, lastLines(readLog(procs.logs.cam), 6));
    end
    if ~bench && toc(t0) > P.link.camStartTimeout
        error('run_real:corners', ['the camera node did not see all corner markers (ids %s) within %g s. ' ...
            'Last camera log lines:\n%s'], mat2str(P.cam.cornerIds), P.link.camStartTimeout, lastLines(readLog(procs.logs.cam), 6));
    end
    pause(0.2);
end
if bench
    say('[ok] bench camera: robots %s reset to their parking slots, echoing their telemetry as camera poses', mat2str(realBots));
    R.checks.bench = true;
else
    say('[ok] camera node sees all %d corner markers', numel(P.cam.cornerIds));
    R.checks.corners = true;
end

% ---- 4. every robot heard (telemetry) and seen (camera)
L = link_open(P, struct('port', procs.port));
ctx('L') = L;
tel = cell(1, nb); cam = cell(1, nb);
t0 = tic;
while true
    link_keepalive(L);
    Rp = link_poll(L);
    for q = 1:numel(Rp.telemetry), k = Rp.telemetry(q).id; if k >= 1 && k <= nb, tel{k} = Rp.telemetry(q); end, end
    for q = 1:numel(Rp.cam), k = Rp.cam(q).bot; if k >= 1 && k <= nb, cam{k} = Rp.cam(q); end, end
    hasTel = ~cellfun(@isempty, tel) & ~cellfun(@(c) ~isempty(c) && strcmp(c.state, 'offline'), tel);
    hasCam = ~cellfun(@isempty, cam);
    if all(hasTel) && all(hasCam), break; end
    if toc(t0) > P.link.readyTimeout
        camHint = 'marker id = robot number, in view, lit?';
        if bench, camHint = sprintf('bench: board power / WiFi, see the bench camera log %s', procs.logs.cam); end
        error('run_real:notReady', ['after %g s: no telemetry from robot(s) %s (power, WiFi, secrets.h MQTT_HOST = ' ...
            'this PC?), no camera pose for robot(s) %s (%s); simulated robots %s ' ...
            '(log %s)'], P.link.readyTimeout, mat2str(find(~hasTel)), mat2str(find(~hasCam)), camHint, mat2str(simBots), ...
            procs.logs.sims);
    end
    pause(0.05);
end
say('[ok] telemetry and camera poses from robots %s', mat2str(1:nb));
R.checks.ready = true;

% ---- 5. real robots on the parking slots (the twin starts robot i on slot P<i>)
bad = {};
R.checks.park = struct('bot', {}, 'dist', {}, 'headingDeg', {});
for k = realBots
    s = Lay.slot(k, :);
    d = hypot(cam{k}.x - s(1), cam{k}.y - s(2));
    dh = abs(rad2deg(atan2(sin(cam{k}.heading - s(3)), cos(cam{k}.heading - s(3)))));
    R.checks.park(end+1) = struct('bot', k, 'dist', d, 'headingDeg', dh);
    if d > P.link.parkTol || dh > P.link.parkTolDeg
        bad{end+1} = sprintf('robot %d is %.0f mm / %.0f deg from P%d (x %.3f, y %.3f, heading %.0f deg); camera sees x %.3f, y %.3f, heading %.0f deg', ...
            k, 1000*d, dh, k, s(1), s(2), rad2deg(s(3)), cam{k}.x, cam{k}.y, rad2deg(cam{k}.heading)); %#ok<AGROW>
    end
end
if ~isempty(bad)
    error('run_real:park', 'place the robots on their parking slots (within %.0f mm, %.0f deg):\n  %s', ...
        1000*P.link.parkTol, P.link.parkTolDeg, strjoin(bad, '\n  '));
end
if ~isempty(realBots)
    say('[ok] real robots %s on their parking slots (worst %.1f mm, %.1f deg)', mat2str(realBots), ...
        1000*max([R.checks.park.dist]), max([R.checks.park.headingDeg]));
end

% ---- 6. run, live view at the real poses
vis = 'off'; if opts.show, vis = 'on'; end
h = world_build(plan, P, struct('visible', vis));
ctx('fig') = h.fig;
% the close callback must not be a nested function: a nested handle stored in a figure keeps this function's
% workspace alive, and the onCleanup (stop + kill) would then never run
set(h.fig, 'CloseRequestFcn', @(src, ~) closeView(src, ctx), 'Name', sprintf('ZADP twin with the real robots (%s)', mode));
world_view(h, 'top');
tAbortChk = tic;
nView = 0; tView = -Inf; nSpots = numel(brain_init(plan, P, opts.seed).spots);
say('running %g s (syncToReal %d); close the view or press Ctrl+C to stop all robots', opts.maxTime, opts.syncToReal);
try
    [W, log, lk] = brain_run_linked(plan, P, struct('link', L, 'maxTime', opts.maxTime, 'seed', opts.seed, ...
        'syncToReal', opts.syncToReal, 'onStep', @onStep, 'abortFcn', @isAborted));
catch runErr
    % rule: a run that ended early prints its end reason (link:offline, link:silent, link:camLost, link:diverged,
    % link:aborted) before the error goes on; brain_run_linked has sent stop already, the cleanup stops again
    say('ended early, all robots stopped: %s: %s', runErr.identifier, runErr.message);
    rethrow(runErr);
end
R.lk = lk; R.log = log; R.mon = log.mon; R.W = W; R.nView = nView;
say('done (end reason %s): %d commands, twin-vs-real error mean %.1f mm, worst %.1f mm, latency mean %.1f ms, hold timeouts %d, twin monitors rr %d pile %d bound %d deadlock %d', ...
    lk.endReason, lk.nCmd, 1000*lk.meanErr, 1000*lk.worstErr, lk.meanLatencyMs, lk.holdTimeouts, log.mon.rrOverlapSteps, ...
    log.mon.pileHitSteps, log.mon.boundSteps, log.mon.deadlocks);
R.camLog = readLog(procs.logs.cam);

    function onStep(W, camNow, ~)
        % live 3D view: piles and beds from the twin, the trucks drawn at the REAL (camera) poses
        tw = toc(t0);
        if tw - tView < opts.viewDt || isempty(h) || ~isgraphics(h.fig), return; end
        tView = tw;
        s = brain_view_state(W, P);
        for kk = 1:nb
            c = camNow{kk};
            if ~isempty(c) && toc(L.t0) - c.tRecv < 0.5, s.pose(kk, :) = [c.x, c.y, c.heading]; end
        end
        world_sync(h, s);
        world_hud(h, W.t, W.truckNo, size(W.piles, 1), nSpots, 0, 0, sprintf('real robots %s (%s)', mat2str(realBots), mode));
        drawnow limitrate;
        nView = nView + 1;
        if ~isempty(opts.viewHook), opts.viewHook(W, h.fig); end
    end

    function a = isAborted()
        a = isKey(ctx, 'abort');                          % set by closeView: brain_run_linked ends with link:aborted
        % rule: every mode has a camera helper (aruco_node.py, --sim in dry, bench_cam.py in bench). When it has exited
        % or logged an error the robots have no camera any more: end the run. The char reason goes into the
        % link:aborted message. Checked every P.link.camCheckDt (the log is small).
        if ~a && toc(tAbortChk) > P.link.camCheckDt
            tAbortChk = tic;
            a = camFault();
        end
    end

    function why = camFault()
        % '' = camera helper healthy, else the reason
        why = '';
        txt = readLog(procs.logs.cam);
        % rule: bench_cam.py logs the word ERROR for any failure (e.g. a board restarted mid-run: the echo cannot
        % correct a boot pose); aruco_node.py logs its own failures as "aruco_node: ERROR". An uncaught Python
        % exception leaves a Traceback in the log.
        word = 'aruco_node: ERROR';
        if bench, word = 'ERROR'; end
        hit = regexp(txt, ['[^\r\n]*(' word '|Traceback)[^\r\n]*'], 'match', 'once');
        if ~isempty(hit)
            why = sprintf('camera helper error in %s: %s', procs.logs.cam, strtrim(hit));
            return
        end
        % rule: link_launch starts the camera helper last, so it is the last process object
        gone = false;
        try, gone = procs.objs{end}.HasExited; catch, end
        if gone
            why = sprintf('camera helper process exited (camera unplugged, video ended, killed?). Last log lines:\n%s', ...
                lastLines(txt, 4));
        end
    end

end

function closeView(src, ctx)
ctx('abort') = true;
delete(src);
end

function shutdownReal(ctx)
% runs on success, error, Ctrl+C: robots to a safe state first (stop twice, 0.2 s apart), then the processes
if isKey(ctx, 'L')
    L = ctx('L');
    try, link_stop_all(L); pause(0.2); link_stop_all(L); catch, end
    link_close(L, false);
end
if isKey(ctx, 'procs')
    left = link_kill(ctx('procs'));
    if ~isempty(left), warning('run_real:procs', 'helper processes still alive: %s', mat2str(left)); end
end
if isKey(ctx, 'fig') && isgraphics(ctx('fig')), delete(ctx('fig')); end
end

function s = readLog(file)
s = '';
if isfile(file), s = fileread(file); end
end

function s = lastLines(txt, n)
l = strsplit(strtrim(txt), newline);
s = strjoin(l(max(1, end-n+1):end), newline);
end
