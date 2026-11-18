function [W, choice] = brain_rank(W)
%BRAIN_RANK  Spots the dispatcher may hand out next (one per zone, or one lane spot), recomputed only
%   when piles/promises changed (W.ver).
%   choice: struct array with k (spot), zone section, blocks (spots it would cut off), lane (bool).
%
%   Rule 1 (fill order): inside each zone, the lowest plan order that is reachable now. A truck may
%   take a later spot in another zone if the first one's zone is locked (brain_step tries the zone
%   choices in order and skips locked zones).
%   Rule 1b (isolation, as in zadp_game's cutsOff): a spot whose pile would cut off the approach of
%   another still-reachable spot (in any zone) waits until that spot is filled. Only when no zone has
%   a spot that cuts nothing off does the spot cutting off the fewest go (lowest order on a tie).
%   Branch spots come after their zone ("branch fills after its zone, like water rising").
%   Rule 4 (spine last): lane spots only when no zone or branch spot is left that a truck could still
%   reach and no zone trip is under way; then deepest first (plan order), one truck at a time.
if W.rankVer == W.ver
    choice = W.rank; return
end
choice = struct('k', {}, 'sec', {}, 'blocks', {}, 'lane', {});
st = {W.spots.status}; kind = {W.spots.kind};
isLane = strcmp(kind, 'lane');
% refresh plans of all pending zone/branch spots (dead ones drop out here)
pend = find(strcmp(st, 'free') & ~isLane);
for k = pend, W = brain_spot_plan(W, k); end
st = {W.spots.status};
live = find(strcmp(st, 'free') & ~isLane & [W.spots.planOK]);
zoneTrip = any(strcmp(st, 'assigned') & ~isLane);
laneTrip = any(strcmp(st, 'assigned') & isLane);
waitingRetry = any(strcmp(st, 'free') & ~isLane & ~[W.spots.planOK]);  % temporarily blocked by a promise
if laneTrip
    % Rule 4: one truck at a time on the spine
elseif ~isempty(live)
    zs = unique([W.spots(live).zone]);
    for z = zs
        inZ = live([W.spots(live).zone] == z);
        zk = inZ(strcmp({W.spots(inZ).kind}, 'zone'));
        if ~isempty(zk), cand = zk; else, cand = inZ; end        % branch after its zone
        [~, o] = sort([W.spots(cand).order]); cand = cand(o);
        best = 0; bestB = Inf;
        for k = cand
            [W, b] = blocksOf(W, k, live);
            if b < bestB, best = k; bestB = b; end
            if b == 0, break; end
        end
        zi = find([W.L.zone.n] == z, 1);
        choice(end+1) = struct('k', best, 'sec', W.L.secZone(zi), 'blocks', bestB, 'lane', false); %#ok<AGROW>
    end
    [~, o] = sortrows([[choice.blocks]' > 0, [W.spots([choice.k]).order]']); choice = choice(o);
    if any([choice.blocks] == 0)
        choice = choice([choice.blocks] == 0);           % blocking spots wait while others can go
    else
        [~, j] = min([choice.blocks]); choice = choice(j);
    end
elseif ~zoneTrip && ~waitingRetry
    pendL = find(strcmp(st, 'free') & isLane);
    [~, o] = sort([W.spots(pendL).order]); pendL = pendL(o);
    for k = pendL, W = brain_spot_plan(W, k); end
    liveL = pendL([W.spots(pendL).planOK] & strcmp({W.spots(pendL).status}, 'free'));
    best = 0; bestB = Inf;
    for k = liveL
        [W, b] = blocksOf(W, k, liveL);
        if b < bestB, best = k; bestB = b; end
        if b == 0, break; end
    end
    if best > 0
        choice(1) = struct('k', best, 'sec', 0, 'blocks', bestB, 'lane', true);
    end
end
W.rank = choice; W.rankVer = W.ver;
end

function [W, b] = blocksOf(W, k, live)
% how many other live spots would lose every plan if a pile sat at spot k
b = 0; c = [W.spots(k).x, W.spots(k).y];
reach = 0.0868 + W.P.brain.pileR + W.P.brain.clearPile + W.P.brain.isoShift + 0.01;
for j = live
    if j == k, continue; end
    R = W.spots(j).plan;
    if isempty(R.axle) || min(hypot(R.axle(:,1)-c(1), R.axle(:,2)-c(2))) > reach, continue; end
    % quick: does the current plan of j still clear a pile at c?
    okNow = true;
    for s = [R.in, R.out]
        [d] = rect_circle_clear(s.poses, c, W.P.brain.pileR, W.P);
        if d < W.P.brain.clearPile + W.P.brain.isoShift, okNow = false; break; end
    end
    if okNow, continue; end
    % piles only get added, so "k cuts j off" stays true for good; "it does not" is re-checked
    % whenever the world near j changed
    % (a "does not" answer is kept until a pile or promise lands near j's planned path: epoch)
    keyB = sprintf('%d_%d', j, k); key = sprintf('%d_%d_%d', j, k, W.spots(j).epoch);
    if isKey(W.isoCache, keyB), ok = false;
    elseif isKey(W.isoCache, key), ok = W.isoCache(key);
    else
        [C, ~] = brain_obstacles(W, j);
        % the real pile lands up to ~1 cm off its spot: test the pile at the spot and shifted by
        % P.brain.isoShift in four directions, so a spot is only called safe if it stays safe
        e = W.P.brain.isoShift;
        X = c + [0 0; e 0; -e 0; 0 e; 0 -e];
        R2 = route_to_spot(W.L, W.P, W.spots(j), [C; X], true);
        W.nPlans = W.nPlans + 1;
        ok = R2.ok; W.isoCache(key) = ok;
        if ~ok, W.isoCache(keyB) = false; end
    end
    if ~ok, b = b + 1; end
end
end
