function [ex, y0] = route_exit_points(L, isLane, yc, pose)
%ROUTE_EXIT_POINTS  Points an exit drives through to reach the OUT lane, and the height y0 where the
%   common tail (route_exit_tail) starts on the OUT lane heading down.
%   Right-zone truck: straight along its branch to the OUT lane centre, turn down there (the turn
%   sweeps back over the IN lane: junction section). Left-zone truck: stops 1.2 cm short of the OUT
%   lane centre, turns down there and slants (5 deg) onto the OUT lane centre 0.13 m lower; a turn
%   on the OUT lane centre itself would sweep 0.087 m east, 2.7 cm into the right zone.
if isLane
    ex = [L.xOut, L.yR]; y0 = L.yR; return
end
if pose(1) < L.xMid
    ex = [L.xOut - 0.012, yc; L.xOut, yc - 0.13]; y0 = yc - 0.13;
else
    ex = [L.xOut, yc]; y0 = yc;
end
end
