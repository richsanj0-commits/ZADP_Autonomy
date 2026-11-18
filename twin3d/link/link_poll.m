function R = link_poll(L)
%LINK_POLL  Read everything that arrived from the bridge since the last call (non-blocking).
%   R = link_poll(L)
%   R.telemetry: struct array {id, seq, encL, encR, x, y, heading, state, tRecv}   (zadp/bot<N>/telemetry)
%   R.cam:       struct array {bot, x, y, heading, t, tRecv}                         (zadp/cam/pose)
%   tRecv = arrival time on the link clock (toc(L.t0)). Missing or null numbers become NaN.
%   The bridge status message (mode) is stored in L.mode on the caller's side via L.st('mode').
R = struct('telemetry', emptyTel(), 'cam', emptyCam());
n = L.tcp.NumBytesAvailable;
if n == 0, return; end
buf = [L.st('buf') char(read(L.tcp, n, 'uint8'))];
k = find(buf == newline);
if isempty(k), L.st('buf') = buf; return; end
lines = strsplit(buf(1:k(end)-1), newline);
L.st('buf') = buf(k(end)+1:end);
tNow = toc(L.t0);
tel = {}; cam = {};
for i = 1:numel(lines)
    if isempty(strtrim(lines{i})), continue; end
    try
        m = jsondecode(lines{i});
    catch
        continue                                    % damaged line: drop
    end
    if ~isfield(m, 'topic') || ~isfield(m, 'payload'), continue; end
    p = m.payload;
    if endsWith(m.topic, '/telemetry')
        id = num(p, 'id'); tk = regexp(m.topic, 'bot(\d+)', 'tokens', 'once');
        if isnan(id) && ~isempty(tk), id = str2double(tk{1}); end
        st = ''; if isfield(p, 'state') && ischar(p.state), st = p.state; end
        tel{end+1} = struct('id', id, 'seq', num(p, 'seq'), 'encL', num(p, 'encL'), 'encR', num(p, 'encR'), ...
            'x', num(p, 'x'), 'y', num(p, 'y'), 'heading', num(p, 'heading'), 'state', st, 'tRecv', tNow); %#ok<AGROW>
    elseif endsWith(m.topic, '/cam/pose')
        cam{end+1} = struct('bot', num(p, 'bot'), 'x', num(p, 'x'), 'y', num(p, 'y'), ...
            'heading', num(p, 'heading'), 't', num(p, 't'), 'tRecv', tNow); %#ok<AGROW>
    elseif endsWith(m.topic, '/bridge/status') && isfield(p, 'mode')
        L.st('mode') = p.mode;
    end
end
if ~isempty(tel), R.telemetry = [tel{:}]; L.st('nTel') = L.st('nTel') + numel(tel); end
if ~isempty(cam), R.cam = [cam{:}]; L.st('nCam') = L.st('nCam') + numel(cam); end
end

function v = num(p, f)
v = NaN;
if isfield(p, f) && isnumeric(p.(f)) && isscalar(p.(f)), v = double(p.(f)); end
end

function s = emptyTel()
s = struct('id', {}, 'seq', {}, 'encL', {}, 'encR', {}, 'x', {}, 'y', {}, 'heading', {}, 'state', {}, 'tRecv', {});
end

function s = emptyCam()
s = struct('bot', {}, 'x', {}, 'y', {}, 'heading', {}, 't', {}, 'tRecv', {});
end
