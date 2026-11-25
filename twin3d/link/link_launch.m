function procs = link_launch(P, opts)
%LINK_LAUNCH  Start the Python helpers as background processes (no window): bridge.py, optionally fake_bot.py and
%   the camera node aruco_node.py, and in broker mode py/mini_broker.py when no broker runs. Used by the tests,
%   demo_link and run_real.
%   procs = link_launch(P, opts)
%   opts.init     N-by-3 start poses [x y heading], row i = robot i (default: the brain's parking slots)
%   opts.mode     'loopback' (default: the bridge is its own router, never touches a broker) or, opt-in,
%                 'broker' (broker or error) / 'auto' (broker if paho-mqtt and a broker answer, else loopback). Broker
%                 mode publishes on the robot topics (zadp/bot<N>/cmd) of P.mqtt.broker:P.mqtt.port: use it only on
%                 purpose; a test may use it only with P.mqtt.port set to a private free port.
%   opts.port     localhost TCP port of the bridge (default: random 18800..18899, so a stale bridge is never reused)
%   opts.ttl      s, every process exits by itself after this (default 900) in case MATLAB dies (0 = never)
%   opts.python   python command (see link_python)
%   opts.bots     robot ids simulated by fake_bot.py (default 1:size(init,1); [] = none). They publish their own
%                 camera pose unless fakeCam is false, and zadp/sim/truth when camNode is 'sim'.
%   opts.fakeCam  opts.bots publish zadp/cam/pose themselves (default true; false when a camera node sees them)
%   opts.simBots  robot ids of a second fake_bot.py: the twin's simulated robots next to real ones in a mixed run
%                 (default []). They always publish their own camera pose and never zadp/sim/truth, so a camera
%                 node never sees them.
%   opts.realBots robot ids that are REAL hardware (default []). Error when opts.bots or opts.simBots contain one;
%                 fake_bot.py gets them as --real and refuses them too, so a fake robot never answers on a real
%                 robot's topics.
%   opts.camNode  '' (default, none), 'sim' (aruco_node.py --sim: simulated camera fed by zadp/sim/truth, same
%                 geometry code as the real camera), 'real' (aruco_node.py on a webcam) or 'bench' (bench_cam.py: the
%                 robots' own telemetry pose republished as camera poses, for BENCH_SIM boards; broker mode only; it
%                 first puts the robots on their start poses opts.init, see bench_cam.py)
%   opts.camBots  robot ids the camera node publishes poses for (default P.cam.robotIds)
%   opts.camArgs  extra command-line arguments for aruco_node.py (e.g. '--camera 1 --width 1920 --height 1080')
%   opts.config   link_config.json to write and pass to every helper (default: a fresh temp file per launch,
%                 written from P by link_write_config, so the helpers always run with these numbers)
%   procs: port, mode ('loopback' or 'broker', as the bridge reported it), broker ('' loopback / 'external' = one
%   that already answered, left alone / 'mini' = mini_broker.py started here), pids (every PID started or reported),
%   logs (bridge, bots, sims, cam, broker), config, python. ALWAYS stop them with link_kill(procs), best from an
%   onCleanup: it stops only what was started here. Processes are started with System.Diagnostics.Process (no
%   console, so no window).
if nargin < 2, opts = struct(); end
def = struct('init', [], 'mode', 'loopback', 'port', [], 'ttl', 900, 'python', '', 'bots', 'all', ...
    'fakeCam', true, 'simBots', [], 'realBots', [], 'camNode', '', 'camBots', P.cam.robotIds, 'camArgs', '', ...
    'config', '');
f = fieldnames(def);
for i = 1:numel(f), if ~isfield(opts, f{i}), opts.(f{i}) = def.(f{i}); end, end
[exe, ~] = link_python(opts);
if isempty(exe), error('link:noPython', 'Python is not available (python on the PATH).'); end
if isempty(opts.port), opts.port = 18800 + randi(99); end
if isempty(opts.init)
    L = brain_layout(load_plan(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'plan', 'plan_arena.json')), P);
    opts.init = L.slot;
end
if ischar(opts.bots), opts.bots = 1:size(opts.init, 1); end
% rule: a robot id is either real hardware or simulated, never both (a fake would answer on the real robot's topics)
both = intersect([opts.bots(:); opts.simBots(:)], opts.realBots(:));
if ~isempty(both)
    error('link:realBot', 'robot(s) %s are real and cannot also be simulated by fake_bot.py', mat2str(both(:)'));
end
assert(isempty(intersect(opts.bots, opts.simBots)), 'link:bots', 'opts.bots and opts.simBots overlap');
pyDir = fullfile(fileparts(mfilename('fullpath')), 'py');
tag = sprintf('%d_%d', opts.port, randi(1e6));
if isempty(opts.config), opts.config = fullfile(tempdir, ['zadp_link_config_' tag '.json']); end
link_write_config(P, opts.config);
procs = struct('port', opts.port, 'mode', '', 'broker', '', 'pids', [], 'objs', {{}}, 'logs', struct(), ...
    'python', exe, 'config', opts.config);
for nm = {'bridge', 'bots', 'sims', 'cam', 'broker'}
    procs.logs.(nm{1}) = fullfile(tempdir, sprintf('zadp_%s_%s.log', nm{1}, tag));
end
cfg = sprintf(' --config "%s"', opts.config);
ttl = '';
if opts.ttl > 0, ttl = sprintf(' --ttl %g', opts.ttl); end
realArg = '';
if ~isempty(opts.realBots), realArg = [' --real ' idList(opts.realBots)]; end

% every process started so far is killed on ANY error below (startup failure, missing log line, a
% second start that throws), so nothing is left running until its --ttl
try
    if strcmp(opts.mode, 'broker')
        % rule: a broker that answers is used as it is (and never stopped here); none answering and the host is this
        % PC -> start mini_broker.py on all interfaces, port P.mqtt.port (link_kill stops it)
        [up, isLocal] = link_broker_status(P);
        if up
            procs.broker = 'external';
        elseif isLocal
            args = sprintf('-u "%s" --port %d%s --log "%s"', fullfile(pyDir, 'mini_broker.py'), P.mqtt.port, ...
                ttl, procs.logs.broker);
            procs = startOne(procs, exe, args, procs.logs.broker, 'listening', 'mini_broker.py');
            procs.broker = 'mini';
        else
            error('link:broker', ['no MQTT broker answers at %s:%d and that host is not this PC (mini_broker.py ' ...
                'can only run here). Start a broker there or set P.mqtt.broker.'], P.mqtt.broker, P.mqtt.port);
        end
    end
    args = sprintf('-u "%s" --port %d%s%s --log "%s" --broker %s --broker-port %d', fullfile(pyDir, 'bridge.py'), ...
        opts.port, ttl, cfg, procs.logs.bridge, P.mqtt.broker, P.mqtt.port);
    if strcmp(opts.mode, 'loopback'), args = [args ' --loopback']; end
    if strcmp(opts.mode, 'broker'), args = [args ' --no-fallback']; end      % broker or nothing
    procs = startOne(procs, exe, args, procs.logs.bridge, 'listening', 'bridge.py');
    tok = regexp(fileread(procs.logs.bridge), 'mode=(\w+)', 'tokens', 'once');
    if isempty(tok), error('link:bridge', 'bridge.py log has no mode line: %s', procs.logs.bridge); end
    procs.mode = tok{1};

    if ~isempty(opts.bots)
        args = [fakeArgs(pyDir, opts, opts.bots, procs.logs.bots, ttl, cfg) realArg];
        if ~opts.fakeCam, args = [args ' --no-cam']; end
        if strcmp(opts.camNode, 'sim'), args = [args ' --truth']; end
        procs = startOne(procs, exe, args, procs.logs.bots, 'attached', 'fake_bot.py');
    end
    if ~isempty(opts.simBots)
        args = [fakeArgs(pyDir, opts, opts.simBots, procs.logs.sims, ttl, cfg) realArg];
        procs = startOne(procs, exe, args, procs.logs.sims, 'attached', 'fake_bot.py (simulated robots)');
    end

    if strcmp(opts.camNode, 'bench')
        % rule: the bench camera talks to the broker (it reads the retained status there) and only with real boards
        assert(strcmp(opts.mode, 'broker'), 'link:benchMode', 'camNode bench needs mode broker');
        args = sprintf('-u "%s" --broker %s:%d%s%s --log "%s" --robots %s', fullfile(pyDir, 'bench_cam.py'), ...
            P.mqtt.broker, P.mqtt.port, ttl, cfg, procs.logs.cam, idList(opts.camBots));
        for i = opts.camBots(:)'
            args = [args sprintf(' --slot %d=%.5f,%.5f,%.5f', i, opts.init(i,:))]; %#ok<AGROW>
        end
        if ~isempty(opts.camArgs), args = [args ' ' opts.camArgs]; end   % e.g. --allow-unflagged
        if ~isempty(opts.camArgs), args = [args ' ' opts.camArgs]; end   % e.g. --allow-unflagged
        procs = startOne(procs, exe, args, procs.logs.cam, 'pid=', 'bench_cam.py');
    elseif ~isempty(opts.camNode)
        args = sprintf('-u "%s" --bridge 127.0.0.1:%d%s%s --log "%s" --robots %s', fullfile(pyDir, 'aruco_node.py'), ...
            opts.port, ttl, cfg, procs.logs.cam, idList(opts.camBots));
        if strcmp(opts.camNode, 'sim'), args = [args ' --sim']; end
        if ~isempty(opts.camArgs), args = [args ' ' opts.camArgs]; end
        procs = startOne(procs, exe, args, procs.logs.cam, 'pid=', 'aruco_node.py');
    end
catch err
    link_kill(procs);
    rethrow(err);
end
procs.pids = unique(procs.pids(~isnan(procs.pids)));
end

function args = fakeArgs(pyDir, opts, ids, logFile, ttl, cfg)
args = sprintf('-u "%s" --bots %s --connect 127.0.0.1:%d%s%s --log "%s"', fullfile(pyDir, 'fake_bot.py'), ...
    idList(ids), opts.port, ttl, cfg, logFile);
for i = ids(:)'
    args = [args sprintf(' --init %d=%.5f,%.5f,%.5f', i, opts.init(i,:))]; %#ok<AGROW>
end
end

function s = idList(ids)
s = strjoin(arrayfun(@num2str, ids(:)', 'UniformOutput', false), ',');
if isempty(s), s = '""'; end
end

function procs = startOne(procs, exe, args, logFile, word, name)
% start one helper, wait (10 s) until its log has the word, add its PID (and the PID it reports itself)
p = System.Diagnostics.Process();
p.StartInfo.FileName = exe;
p.StartInfo.Arguments = args;
p.StartInfo.UseShellExecute = false;
p.StartInfo.CreateNoWindow = true;
p.Start();
procs.objs{end+1} = p;
procs.pids(end+1) = double(p.Id);
t0 = tic;
while ~(isfile(logFile) && contains(fileread(logFile), word))
    if p.HasExited && ~(isfile(logFile) && contains(fileread(logFile), word))     % helper died at start (e.g. exit 1)
        s = '(no log)';
        if isfile(logFile), s = strtrim(fileread(logFile)); s = s(max(1, end-1999):end); end
        error('link:start', '%s stopped at start (exit code %d): %s', name, p.ExitCode, s);
    end
    if toc(t0) > 10
        s = '(no log)';
        if isfile(logFile), s = fileread(logFile); s = s(max(1, end-1999):end); end
        error('link:start', '%s did not start. Log %s:\n%s', name, logFile, s);
    end
    pause(0.1);
end
procs.pids = [procs.pids, str2double(regexp(fileread(logFile), 'pid=(\d+)', 'tokens', 'once'))];
end
