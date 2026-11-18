function n = link_keepalive(L, period)
%LINK_KEEPALIVE  Re-send each robot's last command (same seq) when it is older than period seconds
%   (default 0.1). The robot ignores a repeat except as proof that the link is alive; without any command
%   line for 300 ms it stops (firmware checklist in link/README.md). Returns the number re-sent.
if nargin < 2, period = 0.1; end
last = L.st('last'); n = 0; tNow = toc(L.t0);
for i = 1:numel(last)
    c = last{i};
    if isempty(c) || tNow - c.tSent < period, continue; end
    link_send_cmd(L, c);                      % keeps c.seq, refreshes tSent in L.st('last')
    n = n + 1;
end
end
