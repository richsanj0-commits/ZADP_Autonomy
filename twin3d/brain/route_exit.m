function [steps, ok, why] = route_exit(L, P, pose, R0, isLane, yc, piles, tail)
%ROUTE_EXIT  Exit route from the dump stop: straight back along u to the road point R0 (no turn next
%   to the new pile), then to the OUT lane (junction for zone trips, return-row corner for lane trips),
%   down to the apron and round the loop into the queue tail P3. piles must include the new pile.
%   yc = branch centre line of the zone (ignored for lane trips).
%   tail = the common part from route_exit_tail (computed here when empty).
if nargin < 8 || isempty(tail)
    [~, y0] = route_exit_points(L, isLane, yc, pose);
    [tail, ok, why] = route_exit_tail(L, P, isLane, y0, piles);
    if ~ok, steps = []; return; end
end
so = route_new(pose); ok = true; why = '';
if ~isempty(R0)
    h = atan2(R0(2)-pose(2), R0(1)-pose(1));
    if abs(atan2(sin(h-pose(3)), cos(h-pose(3)))) > 1e-6         % started from a measured pose
        [so, ok, why] = route_steps(so, 'turn', h, piles, L, P);
    end
    if ok, [so, ok, why] = route_steps(so, 'drive', R0, piles, L, P); end
end
% exit points onto the OUT lane (route_exit_points): a left-zone truck turns down short of the OUT
% lane and joins it on a slant, so its turn never sweeps into the right zone (Rule 2)
[ex, y0] = route_exit_points(L, isLane, yc, pose);
for q = 1:size(ex, 1)
    if ok, [so, ok, why] = route_steps(so, 'turn', atan2(ex(q,2)-so.pose(2), ex(q,1)-so.pose(1)), piles, L, P); end
    if ok, [so, ok, why] = route_steps(so, 'drive', ex(q,:), piles, L, P); end
end
if ok && ~isLane, [so, ok, why] = route_steps(so, 'turn', -pi/2, piles, L, P); end
if ok && isLane, [so, ok, why] = route_steps(so, 'turn', 0, piles, L, P, +1); end
steps = [];
if ok, steps = [so.steps, tail.steps]; end
end
