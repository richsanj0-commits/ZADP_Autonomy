function yT = lane_turn_y(L, P, x, y)
%LANE_TURN_Y  Height on a spine lane (centre x) where a robot turns in place between the lane and a
%   branch road at height y: y itself when a full in-place turn there stays inside the site, otherwise
%   the highest point below y where it does (1 mm steps down to the entry line). Returns y when no
%   point fits (the route checks then report why).
%   Rule (general site shapes): a branch road may run close to a site edge across the spine (the bar of
%   an L, a slanted top). The turn between lane and branch is moved down the lane just far enough that
%   the whole turn disc (body reach + P.brain.clearBound) is inside, and the robot then drives a short
%   slant between that point and the branch road. On a map where the turn fits at y nothing changes.
r = L.reach + P.brain.clearBound;
yT = y;
if in_allowed(L, x, y, r), return; end
ys = (y:-0.001:L.yEntry)';
k = find(in_allowed(L, x*ones(size(ys)), ys, r), 1);
if ~isempty(k), yT = ys(k); end
end
