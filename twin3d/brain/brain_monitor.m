function M = brain_monitor(M, W, P)
%BRAIN_MONITOR  Safety monitors on the TRUE poses, one call per brain step (counted, never stopping the run).
%   M = brain_monitor([], W, P) starts them; M = brain_monitor(M, W, P) after every brain_step.
%     (a) robot-robot: oriented body rectangles never overlap (separating-axis test); min gap kept
%     (b) robot-pile : body rectangle never overlaps a pile circle (each pile with its ACTUAL radius); min gap kept
%     (c) site       : the body stays inside plan.boundary, the apron or the gate mouth joining them
%     (e) sections   : every section a robot's true body touches is held by that robot (Rule 3 invariant,
%                      brain_owner_check); this is what makes (a) impossible by construction
%   (d) deadlock is counted by brain_step (Rule 7) and copied in by brain_run.
%   Every event (start of an overlap/violation) is recorded with its time in *Where = [t bot(s) x y th]
%   so a replay can show the count up to t. brain_monitor_collisions(M) = (a) + (b) + (c) events: the one
%   number every viewer shows as "collisions".
%   Speed: a robot whose true pose did not change since the last call (parked, waiting) is not
%   re-checked against piles or the site unless a pile was added; the pile test only looks at piles
%   within body reach; the outline-vs-site test is skipped when the axle is deeper inside one allowed
%   piece than the body reach; ownership is re-checked when a robot moved or W.owner changed. Every
%   robot that moved is checked every step.
nb = numel(W.bots);
poses = reshape([W.bots.s], 1, []); poses = vertcat(poses.pose);
L = W.L;
if isempty(M)
    M = struct('rrOverlapSteps', 0, 'rrOverlapEvents', 0, 'minRR', Inf, ...
        'pileHitSteps', 0, 'pileHitEvents', 0, 'minPile', Inf, ...
        'boundSteps', 0, 'boundEvents', 0, 'maxInside', 0, ...
        'ownerSteps', 0, 'ownerEvents', 0, 'steps', 0);
    M.boundWhere = zeros(0, 5); M.hitWhere = zeros(0, 5); M.rrWhere = zeros(0, 6); M.ownerWhere = zeros(0, 5);
    M.prevRR = false(nb); M.prevPile = false(1, nb); M.prevB = false(1, nb); M.prevOwn = false(1, nb);
    M.lastPose = NaN(nb, 3); M.lastNP = -1; M.lastOwner = []; M.pileD = Inf(1, nb); M.outB = false(1, nb);
    return
end
moved = any(poses ~= M.lastPose, 2)';
newPile = size(W.piles, 1) ~= M.lastNP;
% (a) robot-robot, every pair with a moved robot
for i = 1:nb
    for j = i+1:nb
        if ~(moved(i) || moved(j)), continue; end
        ov = false;
        if hypot(poses(i,1)-poses(j,1), poses(i,2)-poses(j,2)) < 2*L.reach
            [ov, g] = rect_pair(poses(i,:), poses(j,:), P);
            M.minRR = min(M.minRR, g);
        end
        if ov && ~M.prevRR(i,j), M.rrOverlapEvents = M.rrOverlapEvents + 1; M.rrWhere(end+1,:) = [W.t, i, j, poses(i,:)]; end
        M.prevRR(i,j) = ov;
    end
end
M.rrOverlapSteps = M.rrOverlapSteps + nnz(M.prevRR);
% (b) robot-pile
if ~isempty(W.piles)
    rMax = max(W.pileR);
    for i = find(moved | newPile)
        near = abs(W.piles(:,1) - poses(i,1)) < L.reach + rMax & abs(W.piles(:,2) - poses(i,2)) < L.reach + rMax;
        d = Inf;
        if any(near), d = rect_circle_clear(poses(i,:), W.piles(near,:), W.pileR(near)', P); end
        M.pileD(i) = d; M.minPile = min(M.minPile, d);
        hit = d < 0;
        if hit && ~M.prevPile(i), M.pileHitEvents = M.pileHitEvents + 1; M.hitWhere(end+1,:) = [W.t, i, poses(i,:)]; end
        M.prevPile(i) = hit;
    end
end
M.pileHitSteps = M.pileHitSteps + nnz(M.prevPile);
% (c) site
for i = find(moved)
    if in_allowed(L, poses(i,1), poses(i,2), L.reach) || inPiece(L, P, poses(i,:))
        out = false;
    else
        [X, Y] = body_outline(poses(i,:), P, 0, 0.01);
        out = ~all(in_allowed(L, X, Y, 0));
        % ... or an inner (reflex) site corner inside the body (non-convex site only)
        if ~out && ~isempty(L.reflex), out = rect_circle_clear(poses(i,:), L.reflex, 0, P) < 0; end
    end
    if out && ~M.prevB(i), M.boundEvents = M.boundEvents + 1; M.boundWhere(end+1,:) = [W.t, i, poses(i,:)]; end
    M.prevB(i) = out;
end
M.boundSteps = M.boundSteps + nnz(M.prevB);
% (e) section ownership: re-checked whenever a robot moved or any section changed hands
if any(moved) || ~isequal(W.owner, M.lastOwner)
    bad = brain_owner_check(W, P, poses);
else
    bad = M.prevOwn;
end
for i = find(bad & ~M.prevOwn), M.ownerEvents = M.ownerEvents + 1; M.ownerWhere(end+1,:) = [W.t, i, poses(i,:)]; end
M.prevOwn = bad;
M.ownerSteps = M.ownerSteps + nnz(bad);
M.maxInside = max(M.maxInside, sum(in_allowed(L, poses(:,1), poses(:,2), 0, 'site')));
M.steps = M.steps + 1;
M.lastPose = poses; M.lastNP = size(W.piles, 1); M.lastOwner = W.owner;
end

function in = inPiece(L, P, pose)
% the four body corners inside the site polygon and no reflex site corner inside the body -> the whole
% rectangle is inside (exact for any simple polygon: an edge entering the body needs a vertex inside it)
c = cos(pose(3)); s = sin(pose(3));
lx = [P.bot.axleToNose, P.bot.axleToNose, -P.bot.axleToBack, -P.bot.axleToBack]; ly = [-1 1 1 -1]*P.bot.chassisWidth/2;
in = all(in_allowed(L, pose(1) + c*lx - s*ly, pose(2) + s*lx + c*ly, 0, 'site'));
if in && ~isempty(L.reflex), in = rect_circle_clear(pose, L.reflex, 0, P) > 0; end
end
