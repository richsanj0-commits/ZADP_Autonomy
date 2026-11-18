function [tail, ok, why] = route_exit_tail(L, P, isLane, y0, piles)
%ROUTE_EXIT_TAIL  Common end of every exit: zone trips from the OUT lane at height y0 heading down
%   (down the OUT lane to the return row); lane trips from the return-row corner. Then the apron loop:
%   east along the return row, down the east column, west into the queue tail P3. Turn directions in
%   the apron are fixed by the loop design (every turn sweep stays inside the apron).
ok = true; why = '';
if isLane
    tail = route_new([L.xOut, L.yR, 0]);
else
    tail = route_new([L.xOut, y0, -pi/2]);
    [tail, ok, why] = route_steps(tail, 'drive', [L.xOut, L.yR], piles, L, P);
    if ok, [tail, ok, why] = route_steps(tail, 'turn', 0, piles, L, P, +1); end
end
if ok, [tail, ok, why] = route_steps(tail, 'drive', [L.xE, L.yR], piles, L, P); end
if ok, [tail, ok, why] = route_steps(tail, 'turn', -pi/2, piles, L, P, -1); end
if ok, [tail, ok, why] = route_steps(tail, 'drive', [L.xE, L.yQ], piles, L, P); end
if ok, [tail, ok, why] = route_steps(tail, 'turn', pi, piles, L, P, -1); end
if ok, tail = route_steps(tail, 'hold', [], piles, L, P); end
end
