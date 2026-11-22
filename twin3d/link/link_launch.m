function procs = link_launch(P, opts)
%LINK_LAUNCH  Start the Python helpers as background processes (no window): bridge.py, optionally fake_bot.py and
%   the camera node aruco_node.py. Used by the tests, demo_link and run_real.
%   procs = link_launch(P, opts)
%   opts.init    N-by-3 start poses [x y heading] of the fake robots (default: the brain's parking slots)
%   opts.mode    'loopback' (default: the bridge is its own router, never touches a broker) or, opt-in,
%                'broker' (broker or error) / 'auto' (broker if paho-mqtt and a broker answer, else loopback). Broker
%                mode publishes on the REAL robot topics (zadp/bot<N>/cmd): use it only on purpose, never in tests.
%   opts.port    localhost TCP port of the bridge (default: random 18800..18899, so a stale bridge is never reused)
%   opts.ttl     s, every process exits by itself after this (default 900) in case MATLAB dies (0 = never)
%   opts.python  python command (see link_python); opts.noBots true = no fake robots
%   opts.fakeCam fake robots publish zadp/cam/pose themselves (default true; false when a camera node runs)
%   opts.camNode '' (default, none), 'sim' (aruco_node.py --sim: simulated camera fed by the fake robots' true
%                poses, same geometry code as the real camera) or 'real' (aruco_node.py on a webcam)
%   opts.camArgs extra command-line arguments for aruco_node.py (e.g. '--camera 1 --width 1920 --height 1080')
%   opts.config  link_config.json to write and pass to every helper (default: a fresh temp file per launch,
%                written from P by link_write_config, so the helpers always run with these numbers)
%   procs: port, mode ('loopback' or 'broker', as the bridge reported it), pids (every PID started or
%   reported), logs (bridge, bots, cam), config, python. ALWAYS stop them with link_kill(procs), best from an
%   onCleanup. Processes are started with System.Diagnostics.Process (no console, so no window).
if nargin < 2, opts = struct(); end
def = struct('init', [], 'mode', 'loopback', 'port', [], 'ttl', 900, 'python', '', 'noBots', false, ...
    'fakeCam', true, 'camNode', '', 'camArgs', '', 'config', '');
f = fieldnames(def);
for i = 1:numel(f), if ~isfield(opts, f{i}), opts.(f{i}) = def.(f{i}); end, end
[exe, ~] = link_python(opts);
if isempty(exe), error('link:noPython', 'Python is not available (python on the PATH).'); end
if isempty(opts.port), opts.port = 18800 + randi(99); end
if isempty(opts.init)
    L = brain_layout(load_plan(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'plan', 'plan_arena.json')), P);
    opts.init = L.slot;
end
pyDir = fullfile(fileparts(mfilename('fullpath')), 'py');
tag = sprintf('%d_%d', opts.port, randi(1e6));
if isempty(opts.config), opts.config = fullfile(tempdir, ['zadp_link_config_' tag '.json']); end
link_write_config(P, opts.config);
procs = struct('port', opts.port, 'mode', '', 'pids', [], 'objs', {{}}, 'logs', struct(), 'python', exe, ...
    'config', opts.config);
procs.logs.bridge = fullfile(tempdir, ['zadp_bridge_' tag '.log']);
procs.logs.bots = fullfile(tempdir, ['zadp_bots_' tag '.log']);
procs.logs.cam = fullfile(tempdir, ['zadp_cam_' tag '.log']);
cfg = sprintf(' --config "%s"', opts.config);
ttl = '';
if opts.ttl > 0, ttl = sprintf(' --ttl %g', opts.ttl); end

% every process started so far is killed on ANY error below (startup failure, missing log line, a
% second start that throws), so nothing is left running until its --ttl
try
    args = sprintf('-u "%s" --port %d%s%s --log "%s" --broker %s --broker-port %d', fullfile(pyDir, 'bridge.py'), ...
        opts.port, ttl, cfg, procs.logs.bridge, P.mqtt.broker, P.mqtt.port);
    if strcmp(opts.mode, 'loopback'), args = [args ' --loopback']; end
    if strcmp(opts.mode, 'broker'), args = [args ' --no-fallback']; end      % broker or nothing
    procs = startOne(procs, exe, args);
    if ~waitLog(procs.logs.bridge, 'listening', 10)
        error('link:bridge', 'bridge.py did not start. Log %s:\n%s', procs.logs.bridge, tailOf(procs.logs.bridge));
    end
    txt = fileread(procs.logs.bridge);
    tok = regexp(txt, 'mode=(\w+)', 'tokens', 'once');
    if isempty(tok), error('link:bridge', 'bridge.py log has no mode line: %s', procs.logs.bridge); end
    procs.mode = tok{1};
    procs.pids = [procs.pids, str2double(regexp(txt, 'pid=(\d+)', 'tokens', 'once'))];

    if ~opts.noBots
        nb = size(opts.init, 1);
        args = sprintf('-u "%s" --bots %s --connect 127.0.0.1:%d%s%s --log "%s"', fullfile(pyDir, 'fake_bot.py'), ...
            strjoin(arrayfun(@num2str, 1:nb, 'UniformOutput', false), ','), opts.port, ttl, cfg, procs.logs.bots);
        for i = 1:nb
            args = [args sprintf(' --init %d=%.5f,%.5f,%.5f', i, opts.init(i,:))]; %#ok<AGROW>
        end
        if ~opts.fakeCam, args = [args ' --no-cam']; end
        if strcmp(opts.camNode, 'sim'), args = [args ' --truth']; end
        procs = startOne(procs, exe, args);
        if ~waitLog(procs.logs.bots, 'attached', 10)
            error('link:bots', 'fake_bot.py did not start. Log %s:\n%s', procs.logs.bots, tailOf(procs.logs.bots));
        end
        txt = fileread(procs.logs.bots);
        procs.pids = [procs.pids, str2double(regexp(txt, 'pid=(\d+)', 'tokens', 'once'))];
    end

    if ~isempty(opts.camNode)
        args = sprintf('-u "%s" --bridge 127.0.0.1:%d%s%s --log "%s"', fullfile(pyDir, 'aruco_node.py'), ...
            opts.port, ttl, cfg, procs.logs.cam);
        if strcmp(opts.camNode, 'sim'), args = [args ' --sim']; end
        if ~isempty(opts.camArgs), args = [args ' ' opts.camArgs]; end
        procs = startOne(procs, exe, args);
        if ~waitLog(procs.logs.cam, 'pid=', 10)
            error('link:cam', 'aruco_node.py did not start. Log %s:\n%s', procs.logs.cam, tailOf(procs.logs.cam));
        end
        txt = fileread(procs.logs.cam);
        procs.pids = [procs.pids, str2double(regexp(txt, 'pid=(\d+)', 'tokens', 'once'))];
    end
catch err
    link_kill(procs);
    rethrow(err);
end
procs.pids = unique(procs.pids(~isnan(procs.pids)));
end

function procs = startOne(procs, exe, args)
p = System.Diagnostics.Process();
p.StartInfo.FileName = exe;
p.StartInfo.Arguments = args;
p.StartInfo.UseShellExecute = false;
p.StartInfo.CreateNoWindow = true;
p.Start();
procs.objs{end+1} = p;
procs.pids(end+1) = double(p.Id);
end

function ok = waitLog(file, word, timeout)
t0 = tic; ok = false;
while toc(t0) < timeout
    if isfile(file) && contains(fileread(file), word), ok = true; return; end
    pause(0.1);
end
end

function s = tailOf(file)
s = '(no log)';
if isfile(file)
    s = fileread(file);
    if numel(s) > 2000, s = s(end-1999:end); end
end
end
