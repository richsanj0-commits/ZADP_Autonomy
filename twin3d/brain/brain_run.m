function [W, log] = brain_run(plan, P, opts)
%BRAIN_RUN  Run the central brain with 3 robots until every servable spot is filled (or maxTime).
%   [W, log] = brain_run(plan, P, opts)
%   opts.maxTime (s, default 4000), opts.seed (default 1), opts.useCamera (default true),
%   opts.logEvery (s, default 0.1), opts.dt (default P.brain.dt = 0.02 s), opts.verbose (default false).
%   Hooks (used by link/brain_run_linked, empty = none): opts.beforeStep(W) -> W and opts.afterStep(W, Wprev) -> W,
%   called around every brain_step.
%
%   Safety monitors on the TRUE poses, every step (counted, never stopping the run):
%     (a) robot-robot: oriented body rectangles never overlap (separating-axis test); min gap kept
%     (b) robot-pile : body rectangle never overlaps a pile circle; min gap kept
%     (c) site       : the body stays inside plan.boundary, the apron or the gate mouth joining them
%     (d) deadlock   : wait cycle or no progress (brain_step, Rule 7)
%   log: t, pose (N-by-3-by-nb true poses), mode (N-by-nb codes 0 parked 1 run 2 atS), truck (N-by-nb truck number, 0 = none), events,
%        mon (counts and minima), piles (+ pileR, pileRec: one record per pile), coverage, tonnes, trucks, spots, per-robot trips.
if nargin < 3, opts = struct(); end
def = struct('maxTime', 4000, 'seed', 1, 'useCamera', true, 'logEvery', 0.1, 'dt', P.brain.dt, 'verbose', false, ...
    'beforeStep', [], 'afterStep', []);
f = fieldnames(def);
for i = 1:numel(f), if ~isfield(opts, f{i}), opts.(f{i}) = def.(f{i}); end, end
rng(opts.seed);
W = brain_init(plan, P, opts.seed);
W.useCamera = opts.useCamera;
dt = opts.dt; nb = numel(W.bots); L = W.L;

nLog = ceil(opts.maxTime/opts.logEvery) + 2;
log.t = zeros(nLog, 1); log.pose = zeros(nLog, 3, nb); log.mode = zeros(nLog, nb, 'uint8'); log.truck = zeros(nLog, nb, 'uint16');
nl = 0; nextLog = 0;
mon = struct('rrOverlapSteps', 0, 'rrOverlapEvents', 0, 'minRR', Inf, ...
    'pileHitSteps', 0, 'pileHitEvents', 0, 'minPile', Inf, ...
    'boundSteps', 0, 'boundEvents', 0, 'maxInside', 0, 'steps', 0);
mon.boundWhere = zeros(0, 5); mon.hitWhere = zeros(0, 5);
prevRR = false(nb); prevPile = false(1, nb); prevB = false(1, nb);
pairs = nchoosek(1:nb, 2);
tic0 = tic;
while W.t < opts.maxTime && ~W.done
    if ~isempty(opts.beforeStep), W = opts.beforeStep(W); end
    if ~isempty(opts.afterStep), Wprev = W; end
    W = brain_step(W, P, dt);
    if ~isempty(opts.afterStep), W = opts.afterStep(W, Wprev); end
    % ---- monitors on true poses
    poses = reshape([W.bots.s], 1, []); poses = vertcat(poses.pose);
    for q = 1:size(pairs, 1)
        i = pairs(q,1); j = pairs(q,2);
        if hypot(poses(i,1)-poses(j,1), poses(i,2)-poses(j,2)) < 0.25
            [ov, g] = rect_pair(poses(i,:), poses(j,:), P);
            mon.minRR = min(mon.minRR, g);
            if ov
                mon.rrOverlapSteps = mon.rrOverlapSteps + 1;
                if ~prevRR(i,j), mon.rrOverlapEvents = mon.rrOverlapEvents + 1; end
            end
            prevRR(i,j) = ov;
        else
            prevRR(i,j) = false;
        end
    end
    if ~isempty(W.piles)
        for i = 1:nb
            d = rect_circle_clear(poses(i,:), W.piles, W.pileR(:)', P);   % each pile with its ACTUAL radius
            mon.minPile = min(mon.minPile, d);
            hit = d < 0;
            if hit
                mon.pileHitSteps = mon.pileHitSteps + 1;
                if ~prevPile(i), mon.pileHitEvents = mon.pileHitEvents + 1; mon.hitWhere(end+1,:) = [W.t, i, poses(i,:)]; end
            end
            prevPile(i) = hit;
        end
    end
    [X, Y] = body_outline(poses, P, 0, 0.01);
    inA = all(in_allowed(L, X, Y, 0), 2)';
    out = ~inA;
    mon.boundSteps = mon.boundSteps + sum(out);
    mon.boundEvents = mon.boundEvents + sum(out & ~prevB);
    for i = find(out & ~prevB), mon.boundWhere(end+1,:) = [W.t, i, poses(i,:)]; end
    prevB = out;
    nIn = sum(in_allowed(L, poses(:,1), poses(:,2), 0, 'site'));
    mon.maxInside = max(mon.maxInside, nIn);
    mon.steps = mon.steps + 1;
    % ---- sparse log
    if W.t >= nextLog - 1e-9
        nl = nl + 1;
        log.t(nl) = W.t; log.pose(nl,:,:) = reshape(poses', 1, 3, nb);
        log.mode(nl,:) = cellfun(@(m) find(strcmp(m, {'parked','run','atS'})) - 1, {W.bots.mode});
        log.truck(nl,:) = [W.bots.truck];          % truck number carried by each robot (0 = parked, no truck)
        nextLog = nextLog + opts.logEvery;
    end
    if opts.verbose && mod(mon.steps, round(100/dt)) == 0
        fprintf('  t = %6.1f s  piles %3d  wall %.0f s\n', W.t, size(W.piles,1), toc(tic0));
    end
end
log.t = log.t(1:nl); log.pose = log.pose(1:nl,:,:); log.mode = log.mode(1:nl,:); log.truck = log.truck(1:nl,:);
mon.deadlocks = W.deadlocks; mon.deadlock = W.deadlock;
log.mon = mon; log.events = W.events; log.piles = W.piles; log.pileR = W.pileR; log.pileRec = W.pileRec;
[log.coverage, log.tonnes] = brain_coverage(W, plan); log.trucks = W.truckNo; log.spots = W.spots;
log.trips = [W.bots.trips]; log.wall = toc(tic0); log.done = W.done; log.tEnd = W.t;
end
