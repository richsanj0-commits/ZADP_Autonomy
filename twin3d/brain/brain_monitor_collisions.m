function n = brain_monitor_collisions(M, t)
%BRAIN_MONITOR_COLLISIONS  The "collisions" number every viewer shows: robot-robot + robot-pile + off-site
%   events of the monitors M (brain_monitor, or log.mon of brain_run). With t: only events up to time t
%   (replays, world_play).
if nargin < 2
    n = M.rrOverlapEvents + M.pileHitEvents + M.boundEvents; return
end
n = sum(M.rrWhere(:,1) <= t) + sum(M.hitWhere(:,1) <= t) + sum(M.boundWhere(:,1) <= t);
end
