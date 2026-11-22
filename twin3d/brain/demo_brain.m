function varargout = demo_brain(seed)
%DEMO_BRAIN  Full fill of the arena plan by 3 robots under the central brain (seed 1 by default).
%   Saves twin3d\brain\demo_brain.png and prints: piles placed / spots, sim time, collisions,
%   pile hits, boundary violations, deadlocks, trucks numbered, tonnes dumped, coverage, minimum robot-robot gap, trips per robot, and the
%   spots left unfilled with the reason. [W, log] = demo_brain(...) also returns the world and log.
if nargin < 1, seed = 1; end
here = fileparts(mfilename('fullpath'));
P = twin_params();
plan = load_plan(fullfile(fileparts(here), 'plan', 'plan_arena.json'));
[W, log] = brain_run(plan, P, struct('seed', seed));
m = log.mon;
n = numel(W.spots); np = size(W.piles, 1);
fprintf('demo_brain (seed %d): piles placed %d / %d (%.1f %%), sim time %.0f s (site time %.0f s), wall %.0f s\n', ...
    seed, np, n, 100*np/n, log.tEnd, log.tEnd*P.scale.siteTimeFactor, log.wall);
fprintf('  collisions (robot-robot overlaps) %d, pile hits %d, boundary violations %d, deadlocks %d\n', ...
    m.rrOverlapEvents, m.pileHitEvents, m.boundEvents, m.deadlocks);
fprintf('  min robot-robot gap %.1f mm, min robot-pile gap %.1f mm, max robots inside at once %d\n', ...
    1000*m.minRR, 1000*m.minPile, m.maxInside);
fprintf('  trips per robot: %s, worst pile-to-spot error %.1f mm, run ended: %s\n', mat2str(log.trips), ...
    1000*max([W.spots(strcmp({W.spots.status}, 'placed')).err]), W.endReason);
fprintf('  trucks numbered %d (each entry from parking is a new truck), total dumped %.0f t (site scale), coverage %.1f %% of the site area\n', ...
    log.trucks, log.tonnes, 100*log.coverage);
pr = log.pileRec;
fprintf('  pile radius site %.2f..%.2f m (arena %.1f..%.1f mm), height %.2f..%.2f m, payload %.0f..%.0f t\n', ...
    min([pr.r]), max([pr.r]), 1000*min([pr.rArena]), 1000*max([pr.rArena]), min([pr.h]), max([pr.h]), min([pr.payload]), max([pr.payload]));
u = find(~strcmp({W.spots.status}, 'placed'));
for k = u
    s = W.spots(k);
    fprintf('  unfilled: order %3d %-6s (%.3f, %.3f): %s\n', s.order, s.kind, s.x, s.y, s.why);
end
brain_plot(W, log, plan, fullfile(here, 'demo_brain.png'));
fprintf('  saved %s\n', fullfile(here, 'demo_brain.png'));
if nargout, varargout = {W, log}; end
end
