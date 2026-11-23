function [W, log, lk] = brain_run_linked(plan, P, opts)
%BRAIN_RUN_LINKED  Run the brain with the link to the real robots attached. THE TWIN LEADS: the brain keeps
%   running the simulated robots (the proven collision-free twin); whenever a robot's route step changes the
%   step is published as a command (link_from_brain), the real robot executes it with its own encoder
%   controller, and the camera node reports its true pose. The run is paced to the wall clock (sim time =
%   real time), because the real robots cannot go faster.
%   [W, log, lk] = brain_run_linked(plan, P, opts)
%   opts: link (an open link from link_open; default: opens one with opts.host/port and closes it at the end),
%         maxTime (s of sim time, default 30), seed (1), syncToReal (default false), holdTimeout (s, 20),
%         keepAlive (s, heartbeat period to the bridge, default P.link.heartbeat), logDt (s, 0.1), readyTimeout
%         (s, P.link.readyTimeout), camStale (s, 0.2: an older camera pose is logged as NaN and counted in
%         lk.camStale), telStale (s, 2: a robot whose telemetry stopped this long, or that reports 'offline' (its
%         MQTT last will), ends the run with an error), maxLagWarn (s, P.link.hostTimeout: warn when a brain
%         step stalled longer; the bridge keeps repeating the commands for hostTimeout, after that the robots'
%         watchdog stops them), onStep (function onStep(W, cam, tel), called every logDt with the latest camera
%         pose and telemetry per robot (cell arrays, {} = none yet), e.g. a live view), abortFcn (function
%         returning true to end the run with error 'link:aborted', e.g. the view's figure was closed), plus the
%         brain_run options. Any error during the run sends stop to every robot before it is rethrown.
%   syncToReal = true: the twin is HELD at the end of each step (the robot's next step does not start; a dump
%         does not end) until the real robot reports state "arrived" with the seq of that step's command, or
%         holdTimeout passes (then it is released and counted in lk.holdTimeouts).
%   log: as brain_run (twin safety monitors in log.mon).
%   lk: t (N-by-1), twin and real (N-by-3-by-nb; twin = true pose of the simulated robot, real = camera pose of
%         the real robot), err (N-by-nb, m), herr (N-by-nb, rad), cmds (every command with tFirst, tAck,
%         latencyMs), latencyMs, nCmd, nTel, nCam, holdTime (s per robot), holdTimeouts, maxLag (s, longest stall of the brain,
%         e.g. planning, that the clock absorbed), mode (bridge mode), and summary: meanErr, worstErr (m, per robot
%         and overall), meanLatencyMs, maxLatencyMs.
if nargin < 3, opts = struct(); end
def = struct('link', [], 'maxTime', 30, 'seed', 1, 'syncToReal', false, 'holdTimeout', 20, 'keepAlive', P.link.heartbeat, ...
    'logDt', 0.1, 'readyTimeout', P.link.readyTimeout, 'host', '127.0.0.1', 'port', P.link.bridgePort, 'camStale', 0.2, ...
    'telStale', 2, 'maxLagWarn', P.link.hostTimeout, 'onStep', [], 'abortFcn', []);
f = fieldnames(def);
for i = 1:numel(f), if ~isfield(opts, f{i}), opts.(f{i}) = def.(f{i}); end, end
nb = P.bot.count;
own = isempty(opts.link);
if own, L = link_open(P, opts); else, L = opts.link; end
cleanup = onCleanup(@() closeIfOwn(own, L));

% ---- state shared by the hooks (nested functions)
cmdLog = []; latest = cell(1, nb); latestCam = cell(1, nb);
wall0 = NaN; nextLog = 0; maxLag = 0; nStale = zeros(1, nb); holdTime = zeros(1, nb); holdTimeouts = 0; warned = containers.Map('KeyType', 'char', 'ValueType', 'logical');
nLog = ceil(opts.maxTime/opts.logDt) + 2; nl = 0;
lkT = zeros(nLog, 1); lkTwin = nan(nLog, 3, nb); lkReal = nan(nLog, 3, nb);

waitReady();

ro = rmfield(opts, intersect(fieldnames(opts), {'link', 'syncToReal', 'holdTimeout', 'keepAlive', 'readyTimeout', 'host', 'port', 'camStale', 'telStale', 'maxLagWarn', 'onStep', 'abortFcn'}));
ro.beforeStep = @beforeStep; ro.afterStep = @afterStep;
try
    [W, log] = brain_run(plan, P, ro);
catch err
    % any error inside a step (link write failure, bridge gone, planning bug): stop the real robots at once
    % (they would otherwise only stop by their 300 ms watchdog) and report
    try, link_stop_all(L); catch, end
    rethrow(err);
end
link_stop_all(L);                                         % end of the run: robots to a safe state
if maxLag > opts.maxLagWarn
    % a brain step longer than hostTimeout: the bridge stopped repeating the commands, so the real robots' watchdog
    % stopped them mid-drive while the twin went on: the twin-vs-real error then grows
    warning('link:stall', 'brain stalled %.2f s in one step (over %.2f s): the real robots may have stopped by their watchdog', maxLag, opts.maxLagWarn);
end
pump(); pause(0.05); pump();

lk.t = lkT(1:nl); lk.twin = lkTwin(1:nl,:,:); lk.real = lkReal(1:nl,:,:);
dxy = lk.twin(:,1:2,:) - lk.real(:,1:2,:);
lk.err = squeeze(sqrt(sum(dxy.^2, 2)));
if nb == 1, lk.err = lk.err(:); end
dh = lk.twin(:,3,:) - lk.real(:,3,:);
lk.herr = abs(squeeze(atan2(sin(dh), cos(dh))));
if nb == 1, lk.herr = lk.herr(:); end
lk.cmds = cmdLog; lk.nCmd = numel(cmdLog);
lk.nTel = L.st('nTel'); lk.nCam = L.st('nCam'); lk.mode = L.st('mode');
lk.latencyMs = []; if ~isempty(cmdLog), lk.latencyMs = [cmdLog.latencyMs]; lk.latencyMs = lk.latencyMs(~isnan(lk.latencyMs)); end
lk.holdTime = holdTime; lk.holdTimeouts = holdTimeouts; lk.maxLag = maxLag; lk.syncToReal = opts.syncToReal; lk.camStale = nStale;
lk.meanErrBot = mean(lk.err, 1, 'omitnan'); lk.worstErrBot = max(lk.err, [], 1, 'omitnan');
lk.meanErr = mean(lk.err(:), 'omitnan'); lk.worstErr = max(lk.err(:), [], 'omitnan');
lk.meanLatencyMs = mean(lk.latencyMs); lk.maxLatencyMs = max([lk.latencyMs, NaN]);

    function waitReady()
        % every robot must have been heard (telemetry and camera) before the run starts
        t = tic;
        while toc(t) < opts.readyTimeout
            pump();
            if all(~cellfun(@isempty, latest)) && all(~cellfun(@isempty, latestCam)), return; end
            pause(0.02);
        end
        error('link:notReady', 'no telemetry / camera pose from every robot within %g s (telemetry %s, camera %s)', ...
            opts.readyTimeout, mat2str(~cellfun(@isempty, latest)), mat2str(~cellfun(@isempty, latestCam)));
    end

    function W = beforeStep(W)
        if isnan(wall0), wall0 = toc(L.t0); end
        last = L.st('last');
        for k = 1:nb
            c = last{k}; hold = false;
            if opts.syncToReal && ~isempty(c)
                arrived = ~isempty(latest{k}) && latest{k}.seq == c.seq && strcmp(latest{k}.state, 'arrived');
                waited = toc(L.t0) - c.tFirst;
                hold = ~arrived && waited < opts.holdTimeout;
                key = sprintf('%d_%d', k, c.seq);
                if ~arrived && ~hold && ~isKey(warned, key)
                    warned(key) = true; holdTimeouts = holdTimeouts + 1;
                    fprintf('  link: robot %d did not report arrived for seq %d within %g s, twin released\n', k, c.seq, opts.holdTimeout);
                end
            end
            W.hold(k) = hold;
            % time the twin really waited: it finished the step (braking / dump timer out) and the real robot did not
            b = W.bots(k);
            if hold && strcmp(b.mode, 'run') && (b.brake || (strcmp(b.steps(b.k).type, 'dump') && b.timer >= P.truck.dumpTime))
                holdTime(k) = holdTime(k) + P.brain.dt;
            end
        end
    end

    function W = afterStep(W, Wprev)
        cmds = link_from_brain(W, Wprev, L);
        for q = 1:numel(cmds)
            c = cmds(q); c.tAck = NaN; c.latencyMs = NaN;
            if isempty(cmdLog), cmdLog = c; else, cmdLog(end+1) = c; end %#ok<AGROW>
        end
        % pace to the wall clock: sim time may not run ahead of real time
        while toc(L.t0) - wall0 < W.t
            pump(); link_keepalive(L, opts.keepAlive); pause(0.001);
        end
        % a slow step (planning) is a stall: the twin never catches up faster than real time (that would run it ahead
        % of the robots), the clock is moved back by the stall instead
        lag = toc(L.t0) - wall0 - W.t;
        if lag > 0.05, maxLag = max(maxLag, lag); wall0 = wall0 + lag; end
        pump(); link_keepalive(L, opts.keepAlive);
        checkAlive();
        if ~isempty(opts.abortFcn) && opts.abortFcn()
            error('link:aborted', 'run aborted (view closed or abort requested) at t = %.1f s', W.t);
        end
        if W.t >= nextLog - 1e-9 && ~isempty(opts.onStep), opts.onStep(W, latestCam, latest); end
        if W.t >= nextLog - 1e-9
            nextLog = nextLog + opts.logDt;
            if all(~cellfun(@isempty, latestCam)) && nl < nLog
                nl = nl + 1; lkT(nl) = W.t;
                tNow = toc(L.t0);
                for k = 1:nb
                    lkTwin(nl,:,k) = W.bots(k).s.pose;
                    % a camera pose older than camStale (marker hidden, camera node stopped) is not the
                    % robot's pose any more: logged as NaN and counted, never compared
                    if tNow - latestCam{k}.tRecv > opts.camStale
                        nStale(k) = nStale(k) + 1;
                    else
                        lkReal(nl,:,k) = [latestCam{k}.x, latestCam{k}.y, latestCam{k}.heading];
                    end
                end
            end
        end
    end

    function checkAlive()
        % rule: a robot that went offline (MQTT last will) or silent ends the run (the twin would lead a ghost)
        tNow = toc(L.t0);
        for k = 1:nb
            tl = latest{k};
            if isempty(tl), continue; end
            if strcmp(tl.state, 'offline')
                error('link:offline', 'robot %d went offline (MQTT last will) at t = %.1f s', k, tNow);
            end
            if tNow - tl.tRecv > opts.telStale
                error('link:silent', 'no telemetry from robot %d for %.1f s', k, tNow - tl.tRecv);
            end
        end
    end

    function pump()
        R = link_poll(L);
        for q = 1:numel(R.cam)
            k = R.cam(q).bot;
            if k >= 1 && k <= nb, latestCam{k} = R.cam(q); end
        end
        for q = 1:numel(R.telemetry)
            tl = R.telemetry(q); k = tl.id;
            if k < 1 || k > nb, continue; end
            latest{k} = tl;
            % first telemetry carrying the seq of a command = its acknowledgement (round trip)
            if tl.seq > 0 && ~isempty(cmdLog)
                m = find([cmdLog.id] == k & [cmdLog.seq] == tl.seq & isnan([cmdLog.tAck]), 1);
                if ~isempty(m)
                    cmdLog(m).tAck = tl.tRecv; cmdLog(m).latencyMs = 1000*(tl.tRecv - cmdLog(m).tFirst);
                end
            end
        end
    end
end

function closeIfOwn(own, L)
if own, link_close(L); end
end
