% TEST_BRAIN  Phase 4 central brain: 3 robots fill the arena plan, seeds 1, 2, 3.
%   Per seed: zero robot-robot overlaps, zero pile overlaps, zero boundary violations, no deadlock,
%   run ended normally, every pile within 15 mm of its spot, piles >= 85 % of all spots, zone lock
%   never violated (from the log: never two robot positions in one zone polygon), and at some moment
%   two robots inside the site at once.
%   Prints "test_brain PASSED". Takes a few minutes (one full fill per seed).
here = fileparts(mfilename('fullpath'));
P = twin_params();
plan = load_plan(fullfile(here, '..', 'plan', 'plan_arena.json'));
minFrac = 0.85;
tAll = tic;
for seed = 1:3
    [W, log] = brain_run(plan, P, struct('seed', seed));
    m = log.mon; n = numel(W.spots); np = size(W.piles, 1);
    assert(W.done, 'seed %d: run did not end normally (t = %.0f s)', seed, log.tEnd);
    assert(m.rrOverlapSteps == 0, 'seed %d: robot-robot overlap (%d steps)', seed, m.rrOverlapSteps);
    assert(m.pileHitSteps == 0, 'seed %d: robot-pile overlap (%d steps)', seed, m.pileHitSteps);
    assert(m.boundSteps == 0, 'seed %d: body outside the site/apron (%d steps)', seed, m.boundSteps);
    assert(m.deadlocks == 0 && ~m.deadlock, 'seed %d: deadlock', seed);
    err = [W.spots(strcmp({W.spots.status}, 'placed')).err];
    assert(numel(err) == np && all(err <= 0.015), 'seed %d: worst pile-to-spot error %.1f mm', seed, 1000*max(err));
    assert(np >= minFrac*n, 'seed %d: only %d / %d piles (%.1f %%)', seed, np, n, 100*np/n);
    % zone lock from the log: never two robots (axle point = robot position) inside one zone polygon
    nb = size(log.pose, 3);
    inZ = false(numel(log.t), nb, numel(plan.zones));
    for i = 1:nb
        for z = 1:numel(plan.zones)
            zp = plan.zones(z).poly;
            inZ(:,i,z) = inpolygon(log.pose(:,1,i), log.pose(:,2,i), zp(:,1), zp(:,2));
        end
    end
    perZone = squeeze(sum(inZ, 2));
    assert(all(perZone(:) <= 1), 'seed %d: two robots in one zone at t = %.1f s', seed, log.t(find(any(perZone > 1, 2), 1)));
    % concurrency: at least two robots inside the site at once (from the log too)
    B = plan.boundary; inside = false(numel(log.t), nb);
    for i = 1:nb, inside(:,i) = inpolygon(log.pose(:,1,i), log.pose(:,2,i), B(:,1), B(:,2)); end
    assert(m.maxInside >= 2 && any(sum(inside, 2) >= 2), 'seed %d: never two robots inside at once', seed);
    u = find(~strcmp({W.spots.status}, 'placed'));
    fprintf('  seed %d: %d / %d piles (%.1f %%), sim %.0f s, min gap robot %.1f mm, pile %.1f mm, worst pile error %.1f mm, trips %s, unfilled orders %s, %.0f s wall\n', ...
        seed, np, n, 100*np/n, log.tEnd, 1000*m.minRR, 1000*m.minPile, 1000*max(err), mat2str(log.trips), ...
        mat2str([W.spots(u).order]), log.wall);
end
fprintf('  test_brain runtime %.0f s\n', toc(tAll));
disp('test_brain PASSED');
