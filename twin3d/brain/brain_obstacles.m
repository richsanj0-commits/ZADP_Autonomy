function [C, promised] = brain_obstacles(W, k)
%BRAIN_OBSTACLES  Pile centres a new plan for spot k must avoid: placed piles + spots promised to
%   other trucks (assigned, not yet dumped). Spot k itself is excluded.
st = {W.spots.status};
asg = find(strcmp(st, 'assigned')); asg(asg == k) = [];
promised = [[W.spots(asg).x]', [W.spots(asg).y]'];
C = [W.piles; promised];
end
