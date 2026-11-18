function W = brain_init(plan, P, seed)
%BRAIN_INIT  World of the central brain: 3 robots parked in the apron queue, the spot list with
%   status, no piles, free sections, empty event list.
%   W.spots(k): x y zone kind order, status free|assigned|placed|dead, plan (cached trip),
%     why (reason when unreachable), pile (true dump centre), err (pile-to-spot distance), bot.
%   W.bots(i): s (phase 3 robot state), mode parked|run|atS, slot (parking slot 1..3), steps/k (current block), job
%     adv|in|out, spot, R (trip plan), timer, nextCam, trips, waitNeed.
%   Parking: robot i starts in parking slot P<i> (P.brain.parking). Every trip from P1 to the gate is a NEW
%   truck: W.truckNo counts them (1, 2, 3, ...), b.truck / b.payload hold the current one (0 while parked).
%   Piles: W.piles (centres), W.pileR (actual radius of each, arena m) and W.pileRec (one record per pile:
%   truck, bot, spot, order, zone, payload, V/r/h at site and arena scale, t). Payloads come from W.loadStream
%   (own seeded stream, so the robot noise stream is untouched).
%   W.owner(sec) = robot holding that section (0 = free). Zone sections are the zone locks.
if nargin < 3, seed = 1; end
P.brain.pileR = plan.trucks(1).pileR;      % PLANNED (largest) pile radius: all clearances are planned with it
P.pile.nominalPayload = plan.trucks(1).payload;   % t, plan payload the load factor multiplies
W.P = P; W.plan = plan;
W.L = brain_layout(plan, P);
sp = plan.trucks(1).spots;
n = numel(sp);
W.spots = struct('x', {sp.x}, 'y', {sp.y}, 'zone', {sp.zone}, 'kind', {sp.kind}, 'order', {sp.order}, ...
    'status', 'free', 'plan', [], 'planOK', false, 'failVer', -1, 'why', '', 'pile', [NaN NaN], ...
    'err', NaN, 'bot', 0, 'tAssign', NaN, 'tDump', NaN, 'epoch', 0);
W.piles = zeros(0,2); W.pileSpot = zeros(0,1); W.pileR = zeros(0,1);
W.pileRec = struct('truck', {}, 'bot', {}, 'spot', {}, 'order', {}, 'zone', {}, 'payload', {}, 'load', {}, ...
    'V', {}, 'r', {}, 'h', {}, 'Varena', {}, 'rArena', {}, 'hArena', {}, 't', {});
W.truckNo = 0; W.loadStream = RandStream('mt19937ar', 'Seed', 1000 + seed);
W.ver = 0; W.rankVer = -1; W.rank = []; W.isoCache = containers.Map(); W.nPlans = 0;
W.owner = zeros(1, W.L.nSec);
W.slotRes = zeros(1, 3);
W.adv = {[], route_apron(W.L, P, 2, 1), route_apron(W.L, P, 3, 2)};
nb = P.bot.count; assert(nb <= 3, 'the apron has 3 queue slots');
for i = 1:nb
    pose = W.L.slot(i,:);
    b.s = bot_init(P, pose, i);
    b.mode = 'parked'; b.slot = i; b.steps = []; b.k = 0; b.job = ''; b.spot = 0; b.R = [];
    b.timer = 0; b.nextCam = 0; b.trips = 0; b.waitNeed = false(1, W.L.nSec); b.waitSince = NaN;
    b.brake = false; b.cmd = [0 0]; b.inSite = false; b.truck = 0; b.payload = 0; b.load = 0;
    W.bots(i) = b; %#ok<AGROW>
    W.slotRes(i) = i;
    W.owner(brain_need_at(W.L, P, pose)) = i;               % a parked robot holds the sections under it
end
W.t = 0;
W.events = struct('t', {}, 'bot', {}, 'type', {}, 'spot', {}, 'truck', {});
W.nAssigned = 0; W.pickLog = zeros(0, 5); W.done = false; W.endReason = ''; W.useCamera = true;
W.deadlock = false; W.deadlocks = 0; W.cycleTime = 0; W.lastProgress = 0;
W.stats = struct('outRecheckFail', 0, 'exitReplans', 0, 'dispatches', 0);
W.hold = false(1, nb);     % per robot: hold the twin at the end of its step (set by link/brain_run_linked, syncToReal)
end
