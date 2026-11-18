function W = brain_step(W, P, dt)
%BRAIN_STEP  One simulation step of the whole world: dispatch decisions, section (lock) grants and
%   releases, route following for every robot through the phase 3 model (bot_controller / in-place
%   turn -> bot_step -> cam_measure + pose_fusion), dumping, deadlock watch, end of run.
%   Robots drive on their fused estimate (camera on) or odometry (W.useCamera false).
if nargin < 3, dt = P.sim.dt; end
P.brain = W.P.brain;                         % brain numbers incl. pileR from the plan
L = W.L; nb = numel(W.bots);

% ---------------------------------------------------------------- decisions
for i = 1:nb
    b = W.bots(i);
    switch b.mode
        case 'atS'
            % Rule 6: the exit route is granted as a whole (every section it touches, down to the
            % queue tail P3) or the truck waits at its dump stop, inside its own locked zone,
            % where it blocks nobody.
            [W, ok, nd] = tryAcquire(W, i, [b.R.out.need]);
            if ok
                b = W.bots(i);
                b.mode = 'run'; b.job = 'out'; b.steps = b.R.out; b.k = 1; b.brake = false; b.timer = 0;
                W.bots(i) = b; W.slotRes(3) = i;
            else
                W.bots(i).waitNeed = nd;
            end
        case 'parked'
            if b.slot > 1 && W.slotRes(b.slot-1) == 0
                % apron queue: move up one slot when the slot ahead and the way there are free
                steps = W.adv{b.slot};
                [W, ok, nd] = tryAcquire(W, i, [steps.need]);
                if ok
                    W.slotRes(b.slot-1) = i; W.slotRes(b.slot) = 0;
                    b = W.bots(i);
                    b.mode = 'run'; b.job = 'adv'; b.steps = steps; b.k = 1; b.slot = b.slot - 1; b.brake = false;
                    W.bots(i) = b;
                else
                    W.bots(i).waitNeed = nd;
                end
            elseif b.slot == 1 && ~any(strcmp({W.bots.mode}, 'atS'))
                % exits before entries: a truck waiting at its dump stop to leave goes first, so the
                % queue can never starve it (it would otherwise compete with every new dispatch)
                W = dispatch(W, i);
            elseif b.slot == 1
                W.bots(i).waitNeed = false(1, L.nSec);
                W.bots(i).waitNeed(W.L.secZone) = true;     % waits for the trucks in the zones
            end
    end
end

% ---------------------------------------------------------------- motion
anyMoving = false;
for i = 1:nb
    b = W.bots(i);
    cmd = [0 0];
    if strcmp(b.mode, 'run')
        st = b.steps(b.k);
        if W.useCamera, pe = b.s.est; else, pe = b.s.odom; end
        done = false;
        switch st.type
            case 'drive'
                if ~b.brake
                    % the phase 3 go-to-point controller, aimed at the leg's end point; in the last
                    % 5 cm the aim point is slid sideways onto the robot's own track, so it brakes
                    % straight along the leg instead of swinging its nose to fix a mm of offset
                    % (and it never turns around if it passes the point)
                    dirv = [cos(st.h), sin(st.h)];
                    along = (st.p - pe(1:2))*dirv';
                    tgt = st.p;
                    if along < 0.05, tgt = pe(1:2) + max(along, 0)*dirv; end
                    [cmd, arr] = bot_controller(pe, tgt, P);
                    arr = arr || along < P.ctrl.arriveTol;
                    if st.pass
                        % pass-through piece: done once the robot is past the piece's end point
                        dvec = st.p - st.pEnd;
                        done = arr || (pe(1:2) - st.pEnd)*dvec' <= 0;
                    elseif arr
                        b.brake = true; cmd = [0 0];
                    end
                end
                if b.brake, done = max(abs(b.s.v)) < 2e-3; end
            case 'turn'
                if ~b.brake
                    [cmd, arr] = turnCmd(pe(3), st.h, st.dir, st.tol, P);
                    if arr, b.brake = true; cmd = [0 0]; end
                end
                if b.brake, done = max(abs(b.s.v)) < 2e-3; end
            case 'dump'
                b.timer = b.timer + dt;
                if b.timer >= P.truck.dumpTime && ~W.hold(i)   % W.hold: the linked real robot has not arrived yet (link/)
                    W.bots(i) = b;
                    W = placePile(W, i, st);
                    b = W.bots(i);
                    done = true;
                end
            case 'hold'
                done = true;
        end
        % link to the real robots (syncToReal): W.hold(i) keeps the twin at the end of a step until the
        % real robot reports arrived. Pass-through pieces and the dump (handled above) are not held.
        if done && W.hold(i) && ~st.pass && ~strcmp(st.type, 'dump'), done = false; end
        if done
            b.brake = false; b.timer = 0;
            if b.k < numel(b.steps)
                b.k = b.k + 1;
                keep = any(vertcat(b.steps(b.k:end).need), 1);
                W.owner(W.owner == i & ~keep) = 0;      % release what the rest of this block never touches
            else
                W.owner(W.owner == i & ~b.steps(end).need) = 0;
                switch b.job
                    case 'adv', b.mode = 'parked';
                    case 'in',  b.mode = 'atS';
                    case 'out'
                        b.mode = 'parked'; b.slot = 3; b.trips = b.trips + 1;      % back in parking, at P3
                        W.events(end+1) = ev(W.t, i, 'returned', b.spot, b.truck);
                        b.spot = 0; b.truck = 0; b.payload = 0; b.load = 0;   % the truck is gone; the next entry is a new one
                end
                b.job = '';
            end
        end
    end
    b.cmd = cmd;
    % physics (phase 3 model) + camera fix
    if any(cmd ~= 0) || any(b.s.v ~= 0)
        b.s = bot_step(b.s, cmd, P, dt);
        anyMoving = anyMoving || max(abs(b.s.v)) > 1e-3;
    else
        b.s.t = b.s.t + dt;
    end
    if W.useCamera && b.s.t >= b.nextCam - 1e-12
        b.s = pose_fusion(b.s, cam_measure(b.s.pose, P), P);
        b.nextCam = b.nextCam + 1/P.cam.rateHz;
    end
    % entered / exited the site (true axle point)
    inSite = in_allowed(L, b.s.pose(1), b.s.pose(2), 0, 'site');
    if inSite && ~b.inSite, W.events(end+1) = ev(W.t + dt, i, 'entered', b.spot, b.truck); end
    if ~inSite && b.inSite, W.events(end+1) = ev(W.t + dt, i, 'exited', b.spot, b.truck); end
    b.inSite = inSite;
    W.bots(i) = b;
end
W.t = W.t + dt;

% ---------------------------------------------------------------- Rule 7: deadlock watch
% A wait-for graph (robot i waits for a section robot j holds) is rebuilt every step. A cycle that
% lasts deadlockCycleTime, or nobody moving for noProgressTime while spots remain, is a deadlock.
% By design neither can happen: inside the lanes every grant is all-or-nothing (nobody waits while
% holding a lane section); the only holders that wait are parked queue robots (QW, Q2S, Q3S) and
% trucks at their dump stop (their zone). An exit route never needs QW or Q2S, an inbound route
% never needs Q2S or Q3S, and with 3 robots and 3 queue slots a full chain
% stop -> P3 -> P2 -> P1 -> stop would need 4 robots, so every wait chain ends at a robot that moves.
dumping = false;
for i = 1:nb
    b = W.bots(i);
    if strcmp(b.mode, 'run') && strcmp(b.steps(b.k).type, 'dump'), dumping = true; end
end
if anyMoving || dumping, W.lastProgress = W.t; end
G = false(nb);
for i = 1:nb
    nd = W.bots(i).waitNeed;
    if any(nd)
        h = unique(W.owner(nd)); h(h == 0 | h == i) = [];
        G(i, h) = true;
    end
end
if hasCycle(G), W.cycleTime = W.cycleTime + dt; else, W.cycleTime = 0; end
spotsLeft = ~W.done && W.nAssigned > 0;
stuck = spotsLeft && W.t - W.lastProgress > P.brain.noProgressTime;
if W.cycleTime > P.brain.deadlockCycleTime || stuck
    if ~W.deadlock, W.deadlocks = W.deadlocks + 1; end
    W.deadlock = true;
end

% ---------------------------------------------------------------- Rule 8: end of the run
if ~W.done && W.nAssigned == 0 && all(strcmp({W.bots.mode}, 'parked')) && all(sort([W.bots.slot]) == 1:nb)
    [W, ch] = brain_rank(W);
    if isempty(ch)
        W.done = true; W.endReason = 'no spot can be served any more';
    end
end
end

% =====================================================================================
function W = dispatch(W, i)
% Rule 6: a truck leaves the queue head only when its zone lock and every section of its whole
% inbound route are granted at once (plus, for a lane spot, nobody else moving).
W.bots(i).waitNeed = false(1, W.L.nSec);
[W, ch] = brain_rank(W);
if isempty(ch), return; end
nb = numel(W.bots);
firstNeed = [];
for c = ch
    if c.lane
        % Rule 4: one truck at a time on the spine; everybody else parked in the apron
        others = setdiff(1:nb, i);
        if ~all(strcmp({W.bots(others).mode}, 'parked'))
            firstNeed = W.owner > 0 & W.owner ~= i & ~ismember(W.owner, others(strcmp({W.bots(others).mode}, 'parked')));
            break
        end
    else
        % Rule 2: zone lock = the zone section; a truck may take a later spot in another zone
        if W.owner(c.sec) ~= 0 && W.owner(c.sec) ~= i
            if isempty(firstNeed), firstNeed = false(1, W.L.nSec); firstNeed(c.sec) = true; end
            continue
        end
    end
    R = W.spots(c.k).plan;
    [W, ok, nd] = tryAcquire(W, i, [R.in.need]);
    if ok
        W.spots(c.k).status = 'assigned'; W.spots(c.k).bot = i; W.spots(c.k).tAssign = W.t;
        W.nAssigned = W.nAssigned + 1;
        W = brain_piles_changed(W, [W.spots(c.k).x, W.spots(c.k).y]);   % promise: others plan around it
        b = W.bots(i);
        b.mode = 'run'; b.job = 'in'; b.steps = R.in; b.k = 1; b.R = R; b.spot = c.k; b.slot = 0;
        b.brake = false; b.timer = 0; b.waitNeed = false(1, W.L.nSec);
        % every departure from parking P1 is a NEW truck: next running number and its own payload
        % (nominal plan payload x uniform load factor)
        W.truckNo = W.truckNo + 1;
        b.truck = W.truckNo; b.load = W.P.pile.loadMin + (W.P.pile.loadMax - W.P.pile.loadMin)*rand(W.loadStream);
        b.payload = W.P.pile.nominalPayload*b.load;
        W.bots(i) = b; W.slotRes(1) = 0;
        W.stats.dispatches = W.stats.dispatches + 1;
        W.events(end+1) = ev(W.t, i, 'assigned', c.k, W.bots(i).truck);
        W.pickLog(end+1,:) = [W.t, i, c.k, W.spots(c.k).order, c.blocks];
        return
    end
    if isempty(firstNeed), firstNeed = nd; end
end
if ~isempty(firstNeed), W.bots(i).waitNeed = firstNeed; end
end

function [W, ok, missing] = tryAcquire(W, i, needs)
% all-or-nothing grant of a set of sections (no hold-and-wait inside the lanes)
nd = any(reshape(needs, W.L.nSec, []).', 1);
missing = nd & W.owner ~= 0 & W.owner ~= i;
ok = ~any(missing);
if ok
    W.owner(nd) = i;
    W.bots(i).waitNeed = false(1, W.L.nSec);
end
end

function W = placePile(W, i, st)
% Rule 5: after dumpTime the pile exists where the truck really stopped: rear toward the spot,
% pile centre D behind the axle along the truck heading (true pose). It is an obstacle forever.
b = W.bots(i); k = b.spot;
pz = b.s.pose;
c = pz(1:2) - st.D*[cos(pz(3)), sin(pz(3))];
% actual pile of this truck's payload (same maths as the planner), never larger than the planned pile
[r, h, V] = pile_from_payload(b.payload, W.P);
sc = W.P.scale.arenaPerSite;
if r*sc > W.P.brain.pileR, r = W.P.brain.pileR/sc; h = r*tand(W.P.pile.reposeDeg); V = pi*r^2*h/3; end
W.piles(end+1,:) = c; W.pileSpot(end+1,1) = k; W.pileR(end+1,1) = r*sc;
W.pileRec(end+1) = struct('truck', b.truck, 'bot', i, 'spot', k, 'order', W.spots(k).order, 'zone', W.spots(k).zone, ...
    'payload', b.payload, 'load', b.load, 'V', V, 'r', r, 'h', h, 'Varena', V*sc^3, 'rArena', r*sc, 'hArena', h*sc, 't', W.t);
W.spots(k).status = 'placed'; W.spots(k).pile = c; W.spots(k).tDump = W.t;
W.nAssigned = W.nAssigned - 1;
W.spots(k).err = hypot(c(1) - W.spots(k).x, c(2) - W.spots(k).y);
W = brain_piles_changed(W, c);
W.events(end+1) = ev(W.t, i, 'dumped', k, b.truck);
% the exit was planned around the promised pile: re-check it against the real one (it sits exactly
% D behind the true pose, so the first metres away from it are as planned up to the tracking error)
% and re-plan it from the robot's estimated pose when the real pile came closer than half the
% planned clearance (same road point, both turn directions tried)
bad = false;
for s = b.R.out
    if rect_circle_clear(s.poses, c, W.P.brain.pileR, W.P) < 0.5*W.P.brain.clearPile, bad = true; break; end
end
if bad
    W.stats.exitReplans = W.stats.exitReplans + 1;
    sp = W.spots(k); isLane = strcmp(sp.kind, 'lane'); yc = NaN;
    if ~isLane, yc = W.L.zone([W.L.zone.n] == sp.zone).yc; end
    P2 = W.P; P2.brain.clearPile = 0.5*W.P.brain.clearPile;
    [steps, ok] = route_exit(W.L, P2, b.s.est, b.R.R0, isLane, yc, brain_obstacles(W, 0));
    if ok
        W.bots(i).R.out = route_needs(steps, W.L, W.P);
    else
        W.stats.outRecheckFail = W.stats.outRecheckFail + 1;   % kept: the monitor shows what happens
    end
end
end

function [cmd, arrived] = turnCmd(th, h, dir, tolDeg, P)
% in-place turn to heading h in direction dir (+1 CCW, -1 CW), as planned and swept-checked
if dir == 0, dir = 1; end
e = mod(dir*(h - th) + 0.3, 2*pi) - 0.3;           % remaining angle in the planned direction
arrived = abs(e) < deg2rad(tolDeg);
if arrived, cmd = [0 0]; return; end
b = P.bot.track; wmax = P.bot.cruiseSpeed/(b/2);
w = dir*sign(e)*min(wmax, max(P.brain.minTurnRate, P.ctrl.kpHeading*abs(e)));
cmd = [-w*b/2, w*b/2];
end

function c = hasCycle(G)
R = G;
for k = 1:size(G,1), R = R | (double(R)*double(G) > 0); end
c = any(diag(R));
end

function e = ev(t, bot, type, spot, truck)
e = struct('t', t, 'bot', bot, 'type', type, 'spot', spot, 'truck', truck);
end
