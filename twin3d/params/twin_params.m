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
% Effective track = track * trackScale (tyre scrub: a calibrated robot may need > 1, the reference micromouse needed
% 1.14). Used wherever the kinematic track enters: physics (bot_step), odometry (bot_odometry), go-to-point
% (bot_controller), in-place turn (brain_step turnCmd), fake_bot.py and the firmware (config.h TRACK_SCALE must equal it).
P.bot.trackScale    = 1.0;
P.bot.trackEff      = P.bot.track*P.bot.trackScale;   % derived, m
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
P.bot.slipStd       = 0.01;       % fraction, std of the per-wheel ground/rotation scale error (camera off: about 1.7 % / 1.7 deg over 0.2 m, 6 % / 6 deg over 1 m)
P.bot.slipCorrLength = 0.20;      % m of wheel travel over which a slip factor decorrelates (see bot_step)
P.bot.count         = 3;

% ---- Arena ---------------------------------------------------------------
P.arena.w = 1.0;                  % m
P.arena.h = 1.0;                  % m
% Scale is set by the robot: 1 arena m = 200 site m (arena = 200 x 200 m = ~10 acres).
% One robot (0.100 x 0.084 m) stands for one 20.0 x 16.8 m haul truck. See SCALING.md.
P.scale.arenaPerSite = 0.005;     % arena m per site m
P.scale.sitePerArena = 200;       % site m per arena m
% site time factor: cruise at site scale (0.2 m/s x 200 = 40 m/s) over a real loaded hauler (30 km/h);
% site time = sim time x siteTimeFactor (= 4.8, SCALING.md)
P.scale.realHaulSpeed  = 30/3.6;  % m/s
P.scale.siteTimeFactor = P.bot.cruiseSpeed*P.scale.sitePerArena/P.scale.realHaulSpeed;
P.arena.apron = 0.25;             % m, apron strip y in [-apron, 0] below the entry edge (queue + return loop)

% ---- Overhead ArUco camera -----------------------------------------------
P.cam.rateHz        = 30;
P.cam.posStd        = 0.003;      % m
P.cam.headingStdDeg = 1.0;        % deg
P.cam.latency       = 0.05;       % s, image age: cam_measure returns the pose of latency ago, pose_fusion compensates with odometry
P.cam.fusionGain    = 0.3;        % 0..1, weight of a camera fix in pose_fusion (0.3: estimate noise about 1.2 mm / 0.4 deg; latency is compensated, so a low gain costs no lag)
% Real camera node (link/py/aruco_node.py; MATLAB writes these into link/link_config.json, link_write_config)
P.cam.dictionary    = 'DICT_4X4_50';
P.cam.robotIds      = [1 2 3];    % marker id = robot number
P.cam.cornerIds     = [10 11 12 13];
% corner marker CENTRES in arena m (rows = cornerIds), outside the driven area (site [0,1]^2 + apron y >= -0.25)
P.cam.cornerXY      = [-0.06 -0.31; 1.06 -0.31; 1.06 1.06; -0.06 1.06];
P.cam.markerSize    = 0.060;      % m, black square of a robot marker (6x6 modules of 10 mm incl. the black border)
P.cam.cornerMarkerSize = 0.060;   % m, black square of a corner marker
P.cam.quietZone     = 0.010;      % m, white margin around each printed marker (robot label 80 x 80 mm fits the 100 x 84 mm top)
P.cam.markerOffset  = (P.bot.chassisLength/2 - P.bot.axleToBack); % m, robot marker centre AHEAD of the axle (marker centred on the top: 0.026)
P.cam.markerHeight  = 0.050;      % m, robot marker plane above the floor (MEASURE on the robot)
P.cam.cornerHeight  = 0.050;      % m, corner marker plane above the floor (put the corner markers on blocks of the
                                  %    robot-top height: then the homography plane IS the robot marker plane, no parallax)
P.cam.cameraHeight  = 1.20;       % m, lens above the floor (used only when cornerHeight ~= markerHeight)
P.cam.captureDelay  = 0.030;      % s, assumed exposure-to-grab delay of the webcam (image time t = grab time - this)
P.cam.maxAge        = 0.25;       % s, a robot drops a fix whose age (+ link.netDelay) is larger

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
% Pile maths, copied from the planner (zadp_game.html: MATERIALS, first entry "Waste rock /
% overburden (blasted)" = the material of the exported plan; calc(t)):
%   V = payload / rho (t / (t/m^3) = m^3),  r = cbrt(3 V / (pi tan(repose))),  h = r tan(repose)
% There is no shape factor. Used by brain/pile_from_payload.m, at SITE scale (arena = x P.scale.arenaPerSite).
P.pile.density   = 1.8;           % t/m^3, loose density rho (html MATERIALS)
% every new truck carries the nominal plan payload times a load factor drawn uniformly from [loadMin, loadMax]
P.pile.loadMin   = 0.85;
P.pile.loadMax   = 1.00;

% ---- Onboard go-to-point controller (mirror of firmware) -----------------
P.ctrl.kpHeading   = 3.0;         % rad/s of yaw rate per rad of heading error
P.ctrl.kpDistance  = 2.5;         % m/s of forward speed per m of distance (1/s)
P.ctrl.rotateThreshDeg = 25;      % rotate in place above this heading error ...
P.ctrl.rotateExitDeg = 15;        % ... until it is below this one (hysteresis)
P.ctrl.captureRadius = 0.03;      % m, inside this never rotate in place; heading correction fades to 0 at the target
P.ctrl.decelFrac   = 0.5;         % approach braking decel as a fraction of maxAccel
P.ctrl.arriveTol   = 0.005;       % m, on the ESTIMATED pose (true error = this + camera/odometry error, kept under 10 mm)

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
P.brain.apronRowY  = -0.0555;      % m, apron return row (OUT lane -> east). Apron tracking budget (it is cramped):
%   the CW turn at its east corner sweeps the body to 7.5 mm under the apron top (y = 0); a robot on it keeps its
%   true body 9.75 mm above yBand, the queue sections (clearSec 8 mm + 1.75 mm); the queue row is as low as the
%   P3 turn allows (4 mm above the apron bottom)
% PARKING: the three robot parking slots P1, P2, P3 (one source; they are the apron queue slots). Rows are
% [x y heading]; robots park heading west (pi). P1 is under the IN lane (x must equal the plan's IN lane x,
% brain_layout checks it), P3 is the east turn column. A robot waits in a rolling queue P3 -> P2 -> P1
% (see brain/README.md), so the slot does not belong to a robot.
P.brain.parking = [0.44 -0.159 pi; ...      % P1, head, under the IN lane
                   0.75 -0.159 pi; ...      % P2
                   0.90 -0.159 pi];         % P3, tail
P.brain.junctionMargin = 0.13;   % m, junction box J reaches this far below/above the branch centre lines (covers the in-place turns there)
P.brain.junctionSide = 0.03;      % m, ... and this far into each branch mouth
P.brain.exitShort  = 0.012;       % m, a left-zone exit stops this short of the OUT lane centre before turning down ...
P.brain.exitSlantDrop = 0.13;     % m, ... and slants onto the OUT lane centre this much lower (route_exit_points)
P.brain.pieceLen   = 0.05;        % m, long drives are cut into pass-through pieces of at most this length (section release)
P.brain.sweepStep  = 0.005;       % m, spacing of the swept-body samples along a drive
P.brain.slideDist  = 0.05;        % m, in the last this-much of a leg the aim point slides onto the robot's own track
P.brain.lookahead  = 0.15;        % m, before that the robot steers toward a carrot this far ahead on the leg line (cross-track correction)
P.brain.deadlockCycleTime = 10;   % s, a wait cycle lasting this long counts as a deadlock
P.brain.noProgressTime    = 60;   % s, nobody moving while the run is not finished counts as a deadlock

% ---- MQTT ----------------------------------------------------------------
P.mqtt.broker       = 'localhost';
P.mqtt.port         = 1883;
P.mqtt.topicCmd       = 'zadp/bot%d/cmd';
P.mqtt.topicTelemetry = 'zadp/bot%d/telemetry';
P.mqtt.topicCamPose   = 'zadp/cam/pose';

% ---- Link timing (link/TOPICS.md; bridge.py, fake_bot.py and the firmware read these via link_config.json / config.h)
P.link.watchdog    = 0.3;         % s, a robot stops its wheels when no cmd message (new or repeat) arrived this long
P.link.repeatHz    = 10;          % Hz, bridge.py repeats each robot's last command (same seq) at this rate ...
P.link.hostTimeout = 1.0;         % s, ... while the MATLAB client that sent it was heard within this time. Must exceed the
                                  %    longest brain stall (0.4-0.6 s measured): a stall then never stops the robots; a hung
                                  %    MATLAB stops them after hostTimeout + watchdog = 1.3 s, a closed one after watchdog
P.link.heartbeat   = 0.1;         % s, MATLAB sends a heartbeat line to the bridge at least this often (link_keepalive)
P.link.telemetryHz = P.sim.telemetryHz;
P.link.netDelay    = 0.010;       % s, assumed camera-PC -> broker -> robot transit, added to the camera's age field
P.link.bridgePort  = 18830;       % localhost TCP port of bridge.py
P.link.readyTimeout = 15;         % s, run_real waits this long for telemetry + camera poses of every robot
P.link.parkTol     = 0.03;        % m, run_real: a robot must stand this close to its parking slot ...
P.link.parkTolDeg  = 15;          % deg, ... and heading
P.link.camStartTimeout = 20;      % s, run_real: the camera node must see all corner markers within this time
% Linked run (brain_run_linked, run_real): defaults and the limits that END a run (stop to every robot, then error)
P.link.holdTimeout = 20;          % s, syncToReal: the twin waits this long for "arrived" before it is released (counted)
P.link.logDt       = 0.1;         % s, twin-vs-real log period (and onStep period)
P.link.camStale    = 0.2;         % s, a camera pose older than this is not compared (logged as NaN, counted in lk.camStale)
P.link.telStale    = 2;           % s, no telemetry from a robot this long ends the run (link:silent)
P.link.camLossTime = 1.0;         % s, no camera pose of a robot this long ends the run (link:camLost). Above camStale and
                                  %    above the longest brain stall (0.6 s), so a hidden frame or a stall never trips it
P.link.divergeLimit = 0.10;       % m, twin-vs-real position error (fresh camera poses only) above this ...
P.link.divergeTime = 1.0;         % s, ... for this long ends the run (link:diverged). Healthy runs stay below 0.030 m (dry
                                  %    test limit) and 0.032 m (bench, worst measured): 0.10 m is 3 times that, and the 1 s
                                  %    filter drops single bad camera fixes
P.link.camCheckDt  = 1.0;         % s, run_real looks at the camera helper (process alive, no error in its log) this often
end
