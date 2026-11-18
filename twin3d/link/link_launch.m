function procs = link_launch(P, opts)
%LINK_LAUNCH  Start bridge.py and fake_bot.py as background processes (no window) for tests and the demo.
%   procs = link_launch(P, opts)
%   opts.init   N-by-3 start poses [x y heading] of the fake robots (default: the brain's parking slots)
%   opts.mode   'auto' (default: broker if paho-mqtt and a broker are there, else loopback) or 'loopback'
%   opts.port   localhost TCP port of the bridge (default: random 18800..18899, so a stale bridge is never reused)
%   opts.ttl    s, both processes exit by themselves after this (default 900) in case MATLAB dies
%   opts.python python command (see link_python); opts.noBots true = bridge only
%   procs: port, mode ('loopback' or 'broker', as the bridge reported it), pids (every PID started or
%   reported), logs, python. ALWAYS stop them with link_kill(procs), best from an onCleanup.
%   The processes are started with System.Diagnostics.Process (no console, so no window and no PowerShell).
if nargin < 2, opts = struct(); end
def = struct('init', [], 'mode', 'auto', 'port', [], 'ttl', 900, 'python', '', 'noBots', false);
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
procs = struct('port', opts.port, 'mode', '', 'pids', [], 'objs', {{}}, 'logs', struct(), 'python', exe);
procs.logs.bridge = fullfile(tempdir, ['zadp_bridge_' tag '.log']);
procs.logs.bots = fullfile(tempdir, ['zadp_bots_' tag '.log']);

args = sprintf('-u "%s" --port %d --ttl %g --log "%s" --broker %s --broker-port %d', fullfile(pyDir, 'bridge.py'), ...
    opts.port, opts.ttl, procs.logs.bridge, P.mqtt.broker, P.mqtt.port);
if strcmp(opts.mode, 'loopback'), args = [args ' --loopback']; end
procs = startOne(procs, exe, args);
if ~waitLog(procs.logs.bridge, 'listening', 10)
    link_kill(procs);
    error('link:bridge', 'bridge.py did not start. Log: %s', procs.logs.bridge);
end
txt = fileread(procs.logs.bridge);
tok = regexp(txt, 'mode=(\w+)', 'tokens', 'once'); procs.mode = tok{1};
procs.pids = [procs.pids, str2double(regexp(txt, 'pid=(\d+)', 'tokens', 'once'))];

if ~opts.noBots
    nb = size(opts.init, 1);
    args = sprintf('-u "%s" --bots %s --connect 127.0.0.1:%d --ttl %g --log "%s"', fullfile(pyDir, 'fake_bot.py'), ...
        strjoin(arrayfun(@num2str, 1:nb, 'UniformOutput', false), ','), opts.port, opts.ttl, procs.logs.bots);
    for i = 1:nb
        args = [args sprintf(' --init %d=%.5f,%.5f,%.5f', i, opts.init(i,:))]; %#ok<AGROW>
    end
    procs = startOne(procs, exe, args);
    if ~waitLog(procs.logs.bots, 'attached', 10)
        link_kill(procs);
        error('link:bots', 'fake_bot.py did not start. Log: %s', procs.logs.bots);
    end
    txt = fileread(procs.logs.bots);
    procs.pids = [procs.pids, str2double(regexp(txt, 'pid=(\d+)', 'tokens', 'once'))];
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
