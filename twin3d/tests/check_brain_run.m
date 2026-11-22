function [W, log] = check_brain_run(plan, P, seed, minFrac, label)
%CHECK_BRAIN_RUN  One full brain run with every safety and rule assert (used by test_brain and test_maps).
%   Asserts: run ended normally (Rule 8, endReason), zero robot-robot overlaps, zero pile overlaps, zero
%   boundary violations, zero section-ownership violations (Rule 3), no deadlock (Rule 7), no failed exit
%   re-plan, every pile within 15 mm of its spot, piles >= minFrac of all spots, zone lock never violated
%   (from the log: never two robot positions in one zone polygon), lane spots only once no zone spot can be
%   served and no other robot inside the site while a lane truck is out (Rule 4), and at some moment two robots inside the
%   site at once. Prints one line.
if nargin < 5, label = ''; end
[W, log] = brain_run(plan, P, struct('seed', seed));
m = log.mon; n = numel(W.spots); np = size(W.piles, 1);
tag = sprintf('%s seed %d', label, seed);
assert(W.done && strcmp(log.endReason, 'no spot can be served any more'), '%s: run did not end normally (t = %.0f s, %s)', tag, log.tEnd, log.endReason);
assert(m.rrOverlapSteps == 0, '%s: robot-robot overlap (%d steps)', tag, m.rrOverlapSteps);
assert(m.pileHitSteps == 0, '%s: robot-pile overlap (%d steps)', tag, m.pileHitSteps);
assert(m.boundSteps == 0, '%s: body outside the site/apron (%d steps)', tag, m.boundSteps);
assert(m.ownerSteps == 0, '%s: a body reached a section its robot does not hold (%d steps, first at t = %s s)', tag, m.ownerSteps, mat2str(m.ownerWhere(1:min(1, end), 1)));
assert(m.deadlocks == 0 && ~m.deadlock, '%s: deadlock', tag);
assert(log.stats.outRecheckFail == 0, '%s: %d exit re-plans failed', tag, log.stats.outRecheckFail);
err = [W.spots(strcmp({W.spots.status}, 'placed')).err];
assert(numel(err) == np && all(err <= 0.015), '%s: worst pile-to-spot error %.1f mm', tag, 1000*max(err));
assert(np >= minFrac*n, '%s: only %d / %d piles (%.1f %%)', tag, np, n, 100*np/n);
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
assert(all(perZone(:) <= 1), '%s: two robots in one zone at t = %.1f s', tag, log.t(find(any(perZone > 1, 2), 1)));
% Rule 4 from the pick log: once a lane spot was picked, no zone spot is picked later (lanes last)
kind = {W.spots.kind};
isLanePick = strcmp(kind(log.pickLog(:,3)), 'lane');
firstLane = find(isLanePick, 1);
assert(isempty(firstLane) || all(isLanePick(firstLane:end)), '%s: a zone spot was picked after a lane spot', tag);
% ... and while a lane truck is out, every other robot stays in the apron (none inside the site)
B = plan.boundary; inside = false(numel(log.t), nb);
for i = 1:nb, inside(:,i) = inpolygon(log.pose(:,1,i), log.pose(:,2,i), B(:,1), B(:,2)); end
ev = log.events;
for q = find(strcmp({ev.type}, 'assigned'))
    if ~strcmp(W.spots(ev(q).spot).kind, 'lane'), continue; end
    r = find(strcmp({ev.type}, 'returned') & [ev.bot] == ev(q).bot & [ev.t] > ev(q).t, 1);
    tEnd = Inf; if ~isempty(r), tEnd = ev(r).t; end
    k = log.t > ev(q).t + 0.2 & log.t < tEnd - 0.2;
    others = setdiff(1:nb, ev(q).bot);
    assert(~any(any(inside(k, others))), '%s: another robot was inside the site while lane truck %d was out', tag, ev(q).truck);
end
% concurrency: at least two robots inside the site at once (from the log too)
assert(m.maxInside >= 2 && any(sum(inside, 2) >= 2), '%s: never two robots inside at once', tag);
u = find(~strcmp({W.spots.status}, 'placed'));
fprintf('  %s: %d / %d piles (%.1f %%), sim %.0f s, min gap robot %.1f mm, pile %.1f mm, worst pile error %.1f mm, exit re-plans %d, lane trips %d, trips %s, unfilled orders %s, %.0f s wall\n', ...
    tag, np, n, 100*np/n, log.tEnd, 1000*m.minRR, 1000*m.minPile, 1000*max(err), log.stats.exitReplans, sum(isLanePick), ...
    mat2str(log.trips), mat2str([W.spots(u).order]), log.wall);
end
