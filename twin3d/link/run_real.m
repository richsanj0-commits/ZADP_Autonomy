function R = run_real(mode, opts)
%RUN_REAL  One command to run the twin with the REAL robots (the twin leads, the robots follow).
%   R = run_real()            real hardware: broker, 3 ESP32 robots, webcam + ArUco markers
%   R = run_real('dry')       the SAME code path without hardware: loopback bridge, fake_bot.py robots (no camera of
%                             their own) and aruco_node.py --sim (synthetic marker pixels through the real geometry)
%   R = run_real(mode, opts)  opts: maxTime (sim s = wall s, default 600 real / 30 dry), syncToReal (true),
%                             show (live 3D view; default true real / false dry: the view is then built hidden and
%                             still driven, so the path is tested), viewDt (s between view refreshes, 0.2),
%                             camArgs (extra aruco_node.py arguments, e.g. '--camera 1 --width 1920 --height 1080'),
%                             plan (plan file, default plan/plan_arena.json), seed (1),
%                             dry only: init (N-by-3 fake robot start poses, default the parking slots),
%                             simHideCorner (corner ids hidden from the simulated camera, e.g. '11'),
%                             viewHook (function viewHook(W, fig) called after each view refresh; the test uses it
%                             to close the view like a user)
%   Steps (each prints one line; any failure stops with a message that says what to fix):
%     1. writes link/link_config.json from twin_params (the single source; every helper reads it)
%     2. preflight: Python, numpy, paho-mqtt (real), OpenCV with cv2.aruco (real), broker reachable at
%        P.mqtt.broker:P.mqtt.port (real)
%     3. starts bridge.py (real: broker mode, no fallback; dry: loopback) and aruco_node.py (+ fake robots in dry);
%        waits until the camera node has seen all corner markers (P.link.camStartTimeout)
%     4. waits for telemetry AND a camera pose from every robot (P.link.readyTimeout)
%     5. checks every robot i stands on parking slot P<i> (P.link.parkTol, P.link.parkTolDeg) and names any that does not
%     6. brain_run_linked with syncToReal and the live 3D view drawn at the REAL (camera) poses
%   Any error, Ctrl+C or closing the view sends stop to every robot, closes the link and kills every helper process.
%   R: mode, lk, mon (twin safety monitors), log, procs, checks (struct of the preflight / ready / park results), camLog.
if nargin < 1 || isempty(mode), mode = 'real'; end
if nargin < 2, opts = struct(); end
mode = validatestring(mode, {'real', 'dry'});
dry = strcmp(mode, 'dry');
here = fileparts(mfilename('fullpath'));
def = struct('maxTime', 600 - 570*dry, 'syncToReal', true, 'show', ~dry, 'viewDt', 0.2, 'camArgs', '', ...
    'plan', fullfile(fileparts(here), 'plan', 'plan_arena.json'), 'seed', 1, 'init', [], 'simHideCorner', '', 'viewHook', []);
f = fieldnames(def);
for i = 1:numel(f), if ~isfield(opts, f{i}), opts.(f{i}) = def.(f{i}); end, end

P = twin_params();
nb = P.bot.count;
plan = load_plan(opts.plan);
Lay = brain_layout(plan, P);
R = struct('mode', mode, 'lk', [], 'mon', [], 'log', [], 'procs', [], 'checks', struct(), 'camLog', '');
say = @(varargin) fprintf(['run_real(%s): ' varargin{1} '\n'], mode, varargin{2:end});

% ---- 1. config (single source)
cfgFile = link_write_config(P);
say('numbers from twin_params written to %s', cfgFile);

% ---- 2. preflight
[exe, hasPaho, pyVer] = link_python();
if isempty(exe), error('run_real:python', 'Python not found on the PATH (install Python 3 and make "python" work in a shell).'); end
say('[ok] Python %s (%s)', pyVer, exe);
need = {'numpy', 'pip install numpy'};
if ~dry, need(end+1, :) = {'cv2; assert hasattr(cv2, "aruco")', 'pip install opencv-contrib-python'}; end
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
    try
        t = tcpclient(P.mqtt.broker, P.mqtt.port, 'ConnectTimeout', 2); delete(t);
    catch err
        error('run_real:broker', ['no MQTT broker at %s:%d (%s). Start Mosquitto on that PC (mosquitto -v -c ' ...
            'mosquitto.conf with "listener 1883 0.0.0.0" and "allow_anonymous true"), allow TCP 1883 in its firewall, ' ...
            'and set P.mqtt.broker in twin_params.m to its address (the firmware secrets.h MQTT_HOST must be the same).'], ...
            P.mqtt.broker, P.mqtt.port, err.message);
    end
    say('[ok] broker reachable at %s:%d', P.mqtt.broker, P.mqtt.port);
end
R.checks.preflight = true;

% ---- 3. helpers
lo = struct('mode', 'loopback', 'noBots', ~dry, 'fakeCam', false, 'camNode', 'sim', 'camArgs', opts.camArgs, ...
    'config', cfgFile, 'ttl', opts.maxTime + 600, 'port', P.link.bridgePort);
if dry
    lo.port = [];                                         % random port: never collides with a real bridge
    if ~isempty(opts.simHideCorner), lo.camArgs = strtrim([lo.camArgs ' --sim-hide-corner ' opts.simHideCorner]); end
    lo.init = Lay.slot; if ~isempty(opts.init), lo.init = opts.init; end
else
    lo.mode = 'broker'; lo.camNode = 'real';
end
h = [];
% cleanup state in a handle object: an onCleanup must not read the function's own variables (MATLAB may have
% destroyed them already when an error unwinds), so it gets this map, filled as things start
ctx = containers.Map();
cleanup = onCleanup(@() shutdownReal(ctx)); %#ok<NASGU>
procs = link_launch(P, lo);
ctx('procs') = procs;
R.procs = procs;
say('[ok] bridge mode %s on port %d, helpers %s (logs in %s)', procs.mode, procs.port, mat2str(procs.pids), tempdir);
if ~dry && ~strcmp(procs.mode, 'broker'), error('run_real:bridge', 'bridge is not in broker mode (%s)', procs.mode); end
t0 = tic;
while ~contains(readLog(procs.logs.cam), 'homography ready')
    if toc(t0) > P.link.camStartTimeout
        error('run_real:corners', ['the camera node did not see all corner markers (ids %s) within %g s. ' ...
            'Last camera log lines:\n%s'], mat2str(P.cam.cornerIds), P.link.camStartTimeout, lastLines(readLog(procs.logs.cam), 6));
    end
    pause(0.2);
end
say('[ok] camera node sees all %d corner markers', numel(P.cam.cornerIds));
R.checks.corners = true;

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
        error('run_real:notReady', ['after %g s: no telemetry from robot(s) %s (power, WiFi, secrets.h MQTT_HOST = ' ...
            'P.mqtt.broker?), no camera pose for robot(s) %s (marker id = robot number, in view, lit?)'], ...
            P.link.readyTimeout, mat2str(find(~hasTel)), mat2str(find(~hasCam)));
    end
    pause(0.05);
end
say('[ok] telemetry and camera poses from robots %s', mat2str(1:nb));
R.checks.ready = true;

% ---- 5. on the parking slots (the twin starts robot i on slot P<i>)
bad = {};
for k = 1:nb
    s = Lay.slot(k, :);
    d = hypot(cam{k}.x - s(1), cam{k}.y - s(2));
    dh = abs(rad2deg(atan2(sin(cam{k}.heading - s(3)), cos(cam{k}.heading - s(3)))));
    R.checks.park(k) = struct('dist', d, 'headingDeg', dh);
    if d > P.link.parkTol || dh > P.link.parkTolDeg
        bad{end+1} = sprintf('robot %d is %.0f mm / %.0f deg from P%d (x %.3f, y %.3f, heading %.0f deg); camera sees x %.3f, y %.3f, heading %.0f deg', ...
            k, 1000*d, dh, k, s(1), s(2), rad2deg(s(3)), cam{k}.x, cam{k}.y, rad2deg(cam{k}.heading)); %#ok<AGROW>
    end
end
if ~isempty(bad)
    error('run_real:park', 'place the robots on their parking slots (within %.0f mm, %.0f deg):\n  %s', ...
        1000*P.link.parkTol, P.link.parkTolDeg, strjoin(bad, '\n  '));
end
say('[ok] every robot on its parking slot (worst %.1f mm, %.1f deg)', 1000*max([R.checks.park.dist]), max([R.checks.park.headingDeg]));

% ---- 6. run, live view at the real poses
vis = 'off'; if opts.show, vis = 'on'; end
h = world_build(plan, P, struct('visible', vis));
ctx('fig') = h.fig;
% the close callback must not be a nested function: a nested handle stored in a figure keeps this function's
% workspace alive, and the onCleanup (stop + kill) would then never run
set(h.fig, 'CloseRequestFcn', @(src, ~) closeView(src, ctx), 'Name', sprintf('ZADP twin with the real robots (%s)', mode));
world_view(h, 'top');
nView = 0; tView = -Inf; nSpots = numel(brain_init(plan, P, opts.seed).spots);
say('running %g s (syncToReal %d); close the view or press Ctrl+C to stop all robots', opts.maxTime, opts.syncToReal);
[W, log, lk] = brain_run_linked(plan, P, struct('link', L, 'maxTime', opts.maxTime, 'seed', opts.seed, ...
    'syncToReal', opts.syncToReal, 'onStep', @onStep, 'abortFcn', @isAborted));
R.lk = lk; R.log = log; R.mon = log.mon; R.W = W; R.nView = nView;
say('done: %d commands, twin-vs-real error mean %.1f mm, worst %.1f mm, latency mean %.1f ms, hold timeouts %d, twin monitors rr %d pile %d bound %d deadlock %d', ...
    lk.nCmd, 1000*lk.meanErr, 1000*lk.worstErr, lk.meanLatencyMs, lk.holdTimeouts, log.mon.rrOverlapSteps, ...
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
        world_hud(h, W.t, W.truckNo, size(W.piles, 1), nSpots, 0, 0, sprintf('REAL robots (%s camera poses)', mode));
        drawnow limitrate;
        nView = nView + 1;
        if ~isempty(opts.viewHook), opts.viewHook(W, h.fig); end
    end

    function a = isAborted()
        a = isKey(ctx, 'abort');                          % set by closeView: brain_run_linked ends with link:aborted
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
