function L = link_open(P, opts)
%LINK_OPEN  Connect MATLAB to bridge.py (localhost TCP, newline-delimited JSON) and subscribe to
%   the telemetry and camera topics of the contract (link/TOPICS.md).
%   L = link_open(P, opts)   opts.host ('127.0.0.1'), opts.port (18830), opts.timeout (s, default 10: the
%   bridge may still be starting). Uses base MATLAB only (tcpclient, jsonencode/jsondecode).
%   L: tcp (tcpclient), P, t0 (tic, the link clock: toc(L.t0) seconds), st (containers.Map with the mutable
%   state: receive buffer, per-robot command counter, message counters, last commands, 'mode' = bridge mode
%   'loopback' or 'broker' once the first link_poll has seen the bridge status message).
if nargin < 2, opts = struct(); end
def = struct('host', '127.0.0.1', 'port', 18830, 'timeout', 10);
f = fieldnames(def);
for i = 1:numel(f), if ~isfield(opts, f{i}), opts.(f{i}) = def.(f{i}); end, end
t0 = tic; tcp = [];
while isempty(tcp)
    try
        tcp = tcpclient(opts.host, opts.port, 'Timeout', 1, 'ConnectTimeout', 2);
    catch err
        if toc(t0) > opts.timeout
            error('link:connect', 'bridge not reachable at %s:%d (%s)', opts.host, opts.port, err.message);
        end
        pause(0.2);
    end
end
L.tcp = tcp; L.P = P; L.t0 = tic;
L.st = containers.Map();
L.st('buf') = '';
L.st('seq') = zeros(1, P.bot.count);
L.st('mode') = '';
L.st('nSent') = 0; L.st('nTel') = 0; L.st('nCam') = 0;
L.st('last') = cell(1, P.bot.count);            % last command sent to each robot (for keep-alives)
subs = {strrep(P.mqtt.topicTelemetry, 'bot%d', '+'), P.mqtt.topicCamPose};
write(tcp, uint8([jsonencode(struct('subscribe', {subs})) 10]));
end
