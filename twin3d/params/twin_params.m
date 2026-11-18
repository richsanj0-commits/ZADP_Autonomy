function P = twin_params()
%TWIN_PARAMS  Single source of numbers for the twin3d digital twin.
%   P = twin_params() returns one struct. All lengths in metres, angles in
%   radians unless a field name says Deg, times in seconds.
%
%   Physics uses the REAL robot numbers (P.bot). P.truck is VISUAL only.
%
%   Pile helper (see pile_geometry.m): for a conical pile of volume V
%       r = (3*V/(pi*tand(reposeDeg)))^(1/3)
%       h = r*tand(reposeDeg)
%   which satisfies V = pi*r^2*h/3.

% ---- Real robot (used by the physics) -----------------------------------
P.bot.wheelDiameter = 0.034;      % m
P.bot.track         = 0.0753;     % m, wheel centre to wheel centre
P.bot.encoderCPR    = 2772.2;     % counts per wheel rev (4x quadrature)
P.bot.chassisLength = 0.100;      % m
P.bot.chassisWidth  = 0.084;      % m
P.bot.axleToNose    = 0.076;      % m
P.bot.axleToBack    = 0.024;      % m
P.bot.motorNoLoadRpm = 300;       % rpm at the wheel
% derived: wheel surface speed at no load, m/s
P.bot.maxWheelSpeed = P.bot.motorNoLoadRpm/60 * pi * P.bot.wheelDiameter;
P.bot.cruiseSpeed   = 0.20;       % m/s
P.bot.maxAccel      = 0.5;        % m/s^2
P.bot.motorTimeConstant = 0.08;   % s, first-order motor lag
P.bot.slipStd       = 0.01;       % fraction, odometry error per wheel
P.bot.slipCorrLength = 0.20;      % m of wheel travel over which a slip factor decorrelates (see bot_step)
P.bot.count         = 3;

% ---- Arena ---------------------------------------------------------------
P.arena.w = 1.0;                  % m
P.arena.h = 1.0;                  % m
% Scale is set by the robot: 1 arena m = 200 site m (arena = 200 x 200 m = ~10 acres).
% One robot (0.100 x 0.084 m) stands for one 20.0 x 16.8 m haul truck. See SCALING.md.
P.scale.arenaPerSite = 0.005;     % arena m per site m
P.scale.sitePerArena = 200;       % site m per arena m
P.arena.apron = 0.25;             % m, apron strip y in [-apron, 0] below the entry edge (queue + return loop)

% ---- Overhead ArUco camera -----------------------------------------------
P.cam.rateHz        = 30;
P.cam.posStd        = 0.003;      % m
P.cam.headingStdDeg = 1.0;        % deg
P.cam.latency       = 0.05;       % s (not modelled by cam_measure)
P.cam.fusionGain    = 0.5;        % 0..1, weight of a camera fix in pose_fusion

% ---- Simulation ----------------------------------------------------------
P.sim.dt          = 0.01;         % s
P.sim.telemetryHz = 20;

% ---- Haul truck (VISUAL only, drawn over the robot) ----------------------
P.truck.length       = 0.100;     % = robot footprint
P.truck.width        = 0.084;
P.truck.height       = 0.065;
P.truck.bedTiltMaxDeg = 50;
P.truck.dumpTime     = 4;         % s
P.truck.bodyColor    = [0.95 0.75 0.10];
P.truck.bedColor     = [0.90 0.70 0.08];
P.truck.wheelColor   = [0.10 0.10 0.10];

% ---- Pile ----------------------------------------------------------------
P.pile.reposeDeg = 37;
P.pile.color     = [0.55 0.42 0.28];
% Pile maths, copied from the planner (zadp_game.html: MATERIALS lines 1069-1076, first entry "Waste rock /
% overburden (blasted)" = the material of the exported plan; calc(t) lines 1085-1092):
%   V = payload / rho (t / (t/m^3) = m^3),  r = cbrt(3 V / (pi tan(repose))),  h = r tan(repose)
% There is no shape factor. Used by brain/pile_from_payload.m, at SITE scale (arena = x P.scale.arenaPerSite).
P.pile.material  = 'Waste rock / overburden (blasted)';
P.pile.density   = 1.8;           % t/m^3, loose density rho (html line 1070)
% every new truck carries the nominal plan payload times a load factor drawn uniformly from [loadMin, loadMax]
P.pile.loadMin   = 0.85;
P.pile.loadMax   = 1.00;

% ---- Onboard go-to-point controller (mirror of firmware) -----------------
P.ctrl.kpHeading   = 3.0;         % rad/s of yaw rate per rad of heading error
P.ctrl.kpDistance  = 2.5;         % m/s of forward speed per m of distance (1/s)
P.ctrl.rotateThreshDeg = 25;      % rotate in place above this heading error
P.ctrl.decelFrac   = 0.5;         % approach braking decel as a fraction of maxAccel
P.ctrl.arriveTol   = 0.005;       % m, on the ESTIMATED pose (true error = this + camera/odometry error, kept under 10 mm)
P.ctrl.headingTolDeg = 3;

% ---- Central brain (phase 4, see brain/README.md) -------------------------
P.brain.dt         = 0.02;        % s, step of the 3-robot world (50 Hz; the phase 3 model is exact-arc, motor lag 0.08 s)
P.brain.clearPile  = 0.007;       % m, planned body-to-pile clearance (covers tracking error)
P.brain.clearBound = 0.004;       % m, planned body-to-site-edge clearance
P.brain.clearSec   = 0.008;       % m, body inflation when deciding which lane sections a move needs
P.brain.isoShift   = 0.008;       % m, how far a real pile may land off its spot when checking that it cuts nothing off
P.brain.dumpMargin = 0.008;       % m, smallest gap between truck rear and pile edge at the dump stop
P.brain.dumpExtra  = [0 0.01 0.02 0.03 0.045 0.06 0.08 0.10]; % m, extra stop distances tried when the shortest is blocked
P.brain.fanDeg     = [0 -5 5 -10 10 -15 15 -20 20 -25 25 -30 30 -40 40 -50 50 -60 60]; % deg, approach directions (0 = straight from the road)
P.brain.turnTolDeg = 3;           % deg, heading tolerance of an in-place turn
P.brain.dumpTolDeg = 1;           % deg, heading tolerance of the final turn at the dump stop
P.brain.minTurnRate = 0.3;        % rad/s, smallest commanded yaw rate while turning (no creeping)
P.brain.apronRowY  = -0.0545;      % m, apron return row (OUT lane -> east)
% PARKING: the three robot parking slots P1, P2, P3 (one source; they are the apron queue slots). Rows are
% [x y heading]; robots park heading west (pi). P1 is under the IN lane (x must equal the plan's IN lane x,
% brain_layout checks it), P3 is the east turn column. A robot waits in a rolling queue P3 -> P2 -> P1
% (see brain/README.md), so the slot does not belong to a robot.
P.brain.parking = [0.44 -0.157 pi; ...      % P1, head, under the IN lane
                   0.75 -0.157 pi; ...      % P2
                   0.90 -0.157 pi];         % P3, tail
P.brain.deadlockCycleTime = 10;   % s, a wait cycle lasting this long counts as a deadlock
P.brain.noProgressTime    = 60;   % s, nobody moving while spots remain counts as a deadlock

% ---- MQTT ----------------------------------------------------------------
P.mqtt.broker       = 'localhost';
P.mqtt.port         = 1883;
P.mqtt.topicCmd       = 'zadp/bot%d/cmd';
P.mqtt.topicTelemetry = 'zadp/bot%d/telemetry';
P.mqtt.topicCamPose   = 'zadp/cam/pose';
end
