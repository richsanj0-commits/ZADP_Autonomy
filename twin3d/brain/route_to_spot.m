function R = route_to_spot(L, P, spot, piles, checkOnly)
%ROUTE_TO_SPOT  Plan the full trip for one pile spot: from queue head P1 to the dump stop and back
%   through the OUT lane to the queue tail P3. Forward driving plus in-place turns only.
%   spot  struct with x, y, kind ('zone'|'branch'|'lane'), zone
%   piles N-by-2 pile centres that exist (or are promised) before this truck dumps
%   R.ok, R.in (steps P1 -> dump -> hold at the stop), R.out (steps stop -> hold at P3),
%   R.S (stop point), R.u (unit vector spot -> stop), R.D, R.axle (axle samples of the whole trip),
%   R.why (reason of the natural approach when nothing works).
%   checkOnly = true: only R.ok/R.why are needed (no sections, no path samples), used by the isolation test.
%
%   Rule 5 (dumping): the truck stops with its REAR to the spot: S = spot + u*D, heading u, with
%   D = pileR + axleToBack + margin. u points to the side the truck came from: the branch road for
%   zone spots, along the branch toward the spine for branch spots, down the spine for lane spots.
%   Because the robot only drives forward, it drives spot-ward to S, turns 180 deg in place, dumps,
%   and leaves straight along u (no turn next to its own pile).
%   When the straight approach (u exactly perpendicular/along the road, smallest D) is blocked, the
%   brain tries a fan of approach angles (P.brain.fanDeg) and longer stops (P.brain.dumpExtra), both
%   turn directions at every in-place turn, and takes the first plan whose swept body clears every
%   pile by P.brain.clearPile and stays in the site.
if nargin < 5, checkOnly = false; end
r = P.brain.pileR;
c = [spot.x spot.y];
Dmin = r + P.bot.axleToBack + P.brain.dumpMargin;
Ds = Dmin + P.brain.dumpExtra;
R = struct('ok', false, 'in', [], 'out', [], 'S', [], 'u', [], 'D', NaN, 'axle', zeros(0,2), 'why', '', 'R0', []);
pilesOut = [piles; c];

isLane = strcmp(spot.kind, 'lane');
if ~isLane
    zi = find([L.zone.n] == spot.zone, 1);
    Z = L.zone(zi); yc = Z.yc;
    Jin = [L.xIn, yc]; Jout = [L.xOut, yc];
    if strcmp(spot.kind, 'zone'), u0 = [0, sign(yc - c(2))]; else, u0 = [-Z.side, 0]; end
    yRoad = yc;
else
    u0 = [0 -1]; yRoad = L.yLaneRoad;                      % lane spots: from a cross line just above the entry
end

% common prefix (P1 -> junction) and tail (OUT lane -> P3), checked once
st0 = route_new(L.slot(1,:));
if isLane, Jin = [L.xIn, 0]; end                       % lane trips leave the IN lane at the gate mouth
[st0, ok, why] = route_steps(st0, 'turn', pi/2, piles, L, P);
if ok, [st0, ok, why] = route_steps(st0, 'drive', Jin, piles, L, P); end
if ~ok, R.why = ['gate/IN lane ' why]; return; end

% the exit tail is the same for every candidate: check it once
if isLane, yc = NaN; end
[~, y0] = route_exit_points(L, isLane, yc, c);
[tail, ok, why] = route_exit_tail(L, P, isLane, y0, pilesOut);
if ~ok, R.why = ['exit tail ' why]; return; end

firstWhy = '';
for D = Ds
    for a = P.brain.fanDeg
        u = [cosd(a)*u0(1) - sind(a)*u0(2), sind(a)*u0(1) + cosd(a)*u0(2)];
        S = c + D*u;
        R0 = [];
        if abs(u(2)) > 0.05
            t = (yRoad - c(2))/u(2);
            if isLane && abs(u(1)) > 1e-9                    % lane spots: stay inside the spine
                xl = [L.xSpL, L.xSpR] + [1 -1]*(P.bot.chassisWidth/2 + 0.005);
                tx = (xl((u(1) > 0) + 1) - c(1))/u(1);
                t = min(t, tx);
            end
            if t > D + 0.005, R0 = c + t*u; end
        end
        if strcmp(spot.kind, 'zone') && isempty(R0) && abs(u(2)) < 0.3, continue; end
        % the road point must lie on the truck's own branch (Rule 2: no detour through the other zone)
        if ~isLane && ~isempty(R0) && Z.side*(R0(1) - L.xMid) < (L.xSpR - L.xSpL)/2, continue; end
        [ok, why, st] = tryOne(st0, S, R0, u, D);
        if ok
            R.ok = true; R.S = S; R.u = u; R.D = D; R.why = ''; R.R0 = st.R0;
            if checkOnly, return; end
            R.in = route_needs(st.in, L, P); R.out = route_needs(st.out, L, P);
            ax = cell(1, numel(st.in) + numel(st.out));
            q = 0;
            for s = [st.in, st.out]
                q = q + 1; ax{q} = s.poses(1:3:end, 1:2);
            end
            R.axle = vertcat(ax{:});                         % whole trip: any pile near it may break it
            return
        end
        if isempty(firstWhy), firstWhy = why; end
    end
end
R.why = firstWhy;

    function [ok, why, out] = tryOne(st, S, R0, u, D)
        out = [];
        hS = atan2(u(2), u(1));                             % dump heading (away from the pile)
        % quick rejects first: body at the dump pose, then the 180 deg turn there (the tightest move)
        [ok, why] = route_check([S hS], piles, L, P);
        if ~ok, why = ['stop: ' why]; return; end
        if ~isempty(R0), hin = atan2(S(2)-R0(2), S(1)-R0(1)); else, hin = atan2(S(2)-st.pose(2), S(1)-st.pose(1)); end
        [~, ok, why] = route_steps(route_new([S hin]), 'turn', hS, piles, L, P, [], P.brain.dumpTolDeg);
        if ~ok, why = ['turn at stop ' why]; return; end
        if ~isempty(R0), q = R0; else, q = S; end            % first point off the common prefix
        [st, ok, why] = route_steps(st, 'turn', atan2(q(2)-st.pose(2), q(1)-st.pose(1)), piles, L, P);
        if ok, [st, ok, why] = route_steps(st, 'drive', q, piles, L, P); end
        if ok && ~isempty(R0)
            [st, ok, why] = route_steps(st, 'turn', atan2(S(2)-R0(2), S(1)-R0(1)), piles, L, P);
            if ok, [st, ok, why] = route_steps(st, 'drive', S, piles, L, P); end
        end
        if ~ok, why = ['approach ' why]; return; end
        [st, ok, why] = route_steps(st, 'turn', hS, piles, L, P, [], P.brain.dumpTolDeg);
        if ~ok, why = ['turn at stop ' why]; return; end
        st = route_steps(st, 'dump', struct('spot', 0, 'D', D), piles, L, P);
        st = route_steps(st, 'hold', [], piles, L, P);
        inSteps = st.steps;
        % ---- out: straight back along u, then to the OUT lane and the apron return row
        [outSteps, ok, why] = route_exit(L, P, st.pose, R0, isLane, yc, pilesOut, tail);
        if ~ok, why = ['exit ' why]; return; end
        out.in = inSteps; out.out = outSteps; out.R0 = R0;
    end
end
