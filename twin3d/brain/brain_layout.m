function L = brain_layout(plan, P)
%BRAIN_LAYOUT  Fixed geometry of the brain: spine and junction points, apron loop, sections, site region.
%   L = brain_layout(plan, P). Assumes the plan's entry normal is +y (spine vertical, entry on the
%   bottom edge), which is what the exported arena plan has.
%
%   Sections (Rule 3, lane mutex). The whole drivable area is cut into named regions. A robot may
%   start a move only while it holds every section its swept body (inflated by P.brain.clearSec)
%   touches; a section has at most one holder. Every point a body can occupy lies in some section,
%   so two bodies can never overlap (they would both hold the section around the shared point).
%      Z<n>     work zone n with its branch road (x outside the spine)       -> Rule 2 zone lock
%      IN_LOW   IN lane below the junction + the apron gate strip x < spine centre
%      OUT_LOW  OUT lane below the junction + the apron strip under it
%      J        junction: the whole spine width around the branch roads (both lanes: the crossing)
%               and 3 cm into each branch mouth. A robot whose axle is inside a zone polygon always
%               holds that zone: its body reaches >= 0.024 m (+0.008 inflation) past the axle.
%      UP       spine above the junction (lane spots)
%      APR_R    apron return row east of the OUT lane
%      QW, QM, Q2S, Q3S  apron queue row: head slot P1, middle, slot P2, tail slot P3
assert(abs(plan.entry.n(1)) < 1e-9 && plan.entry.n(2) > 0, 'brain_layout: entry normal must be +y');
sp = plan.spine.poly;
L.xSpL = min(sp(:,1)); L.xSpR = max(sp(:,1));
L.xIn = mean(plan.spine.inLine(:,1)); L.xOut = mean(plan.spine.outLine(:,1));
L.xMid = (L.xSpL + L.xSpR)/2;
L.yEntry = plan.entry.pt(2);
L.yLaneRoad = L.yEntry + 0.05;                                % cross line the lane-spot trips start from
L.boundary = plan.boundary;
L.apron = P.arena.apron; L.arenaW = P.arena.w;

% zones: side of the spine, branch centre line, section id
nz = numel(plan.zones);
for i = 1:nz
    z = plan.zones(i);
    L.zone(i).n = z.n;
    L.zone(i).side = sign(mean(z.poly(:,1)) - L.xMid);          % -1 left of the spine, +1 right
    if ~isempty(z.branch)
        b = z.branch{1}; L.zone(i).yc = (min(b(:,2)) + max(b(:,2)))/2;
    else
        L.zone(i).yc = plan.entry.pt(2) + z.branchDepth;
    end
    L.zone(i).poly = z.poly; L.zone(i).branch = z.branch;
end
yc = [L.zone.yc];
L.yJ1 = min(yc) - P.brain.junctionMargin; L.yJ2 = max(yc) + P.brain.junctionMargin;   % junction box (covers the in-place turns there)
L.jSide = P.brain.junctionSide;                                % ... and that far into each branch mouth (see below)
% body reach from the axle point (farthest body corner): every "is anything within reach" prefilter uses it
L.reach = hypot(P.bot.axleToNose, P.bot.chassisWidth/2);

% apron loop: queue row (west) -> IN lane -> site -> OUT lane -> return row (east) -> down -> queue tail
L.yR = P.brain.apronRowY; L.yQ = P.brain.parking(1,2);
L.slotX = P.brain.parking(:,1)';
assert(abs(L.slotX(1) - L.xIn) < 1e-6 && all(P.brain.parking(:,2) == L.yQ) && all(P.brain.parking(:,3) == pi), ...
    'brain_layout: P.brain.parking must be one row heading west, P1 under the plan IN lane');
L.xE = L.slotX(3);                                             % east column = tail slot
L.slot = P.brain.parking;                                       % parking P1..P3, parked heading west
L.yBand = (L.yR + L.yQ)/2;                                     % between the apron rows (return row / queue row)
% queue row sections: QM ends one clearSec short of the inflated nose of a robot parked at P2 (heading
% west), Q3S starts halfway between a P2 robot's back and a P3 robot's nose
L.xQM = [L.xMid, L.slotX(2) - P.bot.axleToNose - 2*P.brain.clearSec];
L.xQ3 = (L.slotX(2) + P.bot.axleToBack + L.slotX(3) - P.bot.axleToNose)/2;

L.secNames = {'Z1','Z2','IN_LOW','OUT_LOW','J','UP','APR_R','QW','QM','Q2S','Q3S'};
L.nSec = numel(L.secNames);
L.secZone = zeros(1, nz);
for i = 1:nz, L.secZone(i) = 1 + (L.zone(i).side > 0); end      % Z1 = left side, Z2 = right side

% allowed region pieces (Monitor c): site polygon, apron strip, gate mouth joining them
L.apronRect = [0, L.arenaW, -L.apron, 0];
L.mouthRect = [L.xSpL, L.xSpR, -L.apron, L.yEntry + 0.03];
% site polygon: any simple polygon (convex or not, e.g. an L). Stored as edges for the exact
% point-in-polygon + distance-to-edge test of in_allowed, plus inward edge normals (the fast test,
% exact only when the polygon is convex: L.convex says which in_allowed uses).
B = L.boundary; n = size(B,1);
cw = sum((B([2:n 1],1)-B(:,1)).*(B([2:n 1],2)+B(:,2))) > 0;
L.bndN = zeros(n,2); L.bndD = zeros(n,1);
for i = 1:n
    e = B(mod(i,n)+1,:) - B(i,:); e = e/norm(e);
    nr = [-e(2), e(1)]; if cw, nr = -nr; end
    L.bndN(i,:) = nr; L.bndD(i) = nr*B(i,:)';
end
L.bndA = B; L.bndB = B([2:n 1],:);                             % edge i runs bndA(i,:) -> bndB(i,:)
% reflex (inward-pointing) corners: a body rectangle whose corners are all inside can still have such a
% corner poking into it, so route_check and the site monitor also keep the body clear of these points
% (empty for a convex site)
e1 = B - B([n 1:n-1],:); e2 = B([2:n 1],:) - B;
cr = e1(:,1).*e2(:,2) - e1(:,2).*e2(:,1);
if cw, cr = -cr; end                                           % > 0 convex corner (CCW), < 0 reflex
L.reflex = B(cr < -1e-12, :);
L.convex = isempty(L.reflex);
end
