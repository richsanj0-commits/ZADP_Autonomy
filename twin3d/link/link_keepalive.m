function n = link_keepalive(L, period)
%LINK_KEEPALIVE  Heartbeat to bridge.py: send {"heartbeat": t} when nothing was sent for period seconds
%   (default P.link.heartbeat = 0.1). The bridge repeats each robot's last command at P.link.repeatHz only while
%   this MATLAB client was heard within P.link.hostTimeout (TOPICS.md "Timing"); without heartbeats the repeats
%   stop and the robots' watchdog (P.link.watchdog) stops them. Returns 1 when a heartbeat was sent, else 0.
if nargin < 2 || isempty(period), period = L.P.link.heartbeat; end
tNow = toc(L.t0); n = 0;
if tNow - L.st('hb') < period, return; end
write(L.tcp, uint8([jsonencode(struct('heartbeat', round(tNow, 3))) 10]));
L.st('hb') = tNow; n = 1;
end
