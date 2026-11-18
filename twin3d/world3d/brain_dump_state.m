function d = brain_dump_state(W, P)
%BRAIN_DUMP_STATE  Per-robot view state of a brain world W, cheap enough to call every step.
%   d.pose (nb-by-3 true poses), d.mode (nb: 0 parked, 1 run, 2 at the dump stop), d.truck (nb truck numbers, 0 =
%   none), d.bed (nb bed angles, deg), d.dump (nb-by-6 rows [x y r h key frac]: the pile being dumped by that
%   robot right now, centre / radius / height in arena m, spot number, visible fraction; all zero when it does not
%   dump).
%   A dump in progress: the robot is in its 'dump' step with b.timer seconds elapsed of P.truck.dumpTime (dump_profile
%   gives bed angle and pile fraction); the pile will be centred D behind the axle (brain_step>placePile) and has
%   the size of this truck's payload (pile_from_payload), capped at the planned radius as placePile does.
nb = numel(W.bots);
d.pose = zeros(nb, 3); d.mode = zeros(nb, 1); d.truck = zeros(nb, 1); d.bed = zeros(nb, 1); d.dump = zeros(nb, 6);
names = {'parked', 'run', 'atS'};
for i = 1:nb
    b = W.bots(i);
    d.pose(i,:) = b.s.pose; d.truck(i) = b.truck; d.mode(i) = find(strcmp(b.mode, names)) - 1;
    if strcmp(b.mode, 'run') && strcmp(b.steps(b.k).type, 'dump')
        [d.bed(i), pf] = dump_profile(b.timer/P.truck.dumpTime, P);
        if strcmp(W.spots(b.spot).status, 'placed'), continue; end      % placed this very step
        [~, ~, ~, A] = pile_from_payload(b.payload, P);
        rA = min(A.r, W.P.brain.pileR); hA = rA*tand(P.pile.reposeDeg);
        pz = b.s.pose;
        c = pz(1:2) - b.steps(b.k).D*[cos(pz(3)), sin(pz(3))];
        d.dump(i,:) = [c, rA, hA, b.spot, pf];
    end
end
end
