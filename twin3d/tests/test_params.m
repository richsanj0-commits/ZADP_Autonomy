% TEST_PARAMS  Checks twin_params and pile_geometry. Prints "test_params PASSED".
P = twin_params();

spec = struct( ...
    'bot',   {{'wheelDiameter','track','encoderCPR','chassisLength','chassisWidth', ...
               'axleToNose','axleToBack','motorNoLoadRpm','maxWheelSpeed','cruiseSpeed', ...
               'maxAccel','motorTimeConstant','slipStd','count','trackScale','trackEff'}}, ...
    'arena', {{'w','h'}}, ...
    'scale', {{'arenaPerSite','sitePerArena'}}, ...
    'cam',   {{'rateHz','posStd','headingStdDeg','latency','fusionGain','dictionary','robotIds','cornerIds', ...
               'cornerXY','markerSize','cornerMarkerSize','quietZone','markerOffset','markerHeight','cornerHeight', ...
               'cameraHeight','captureDelay','maxAge'}}, ...
    'link',  {{'watchdog','repeatHz','hostTimeout','heartbeat','telemetryHz','netDelay','bridgePort', ...
               'readyTimeout','parkTol','parkTolDeg','camStartTimeout'}}, ...
    'sim',   {{'dt','telemetryHz'}}, ...
    'truck', {{'length','width','height','bedTiltMaxDeg','dumpTime', ...
               'bodyColor','bedColor','wheelColor'}}, ...
    'pile',  {{'reposeDeg','color'}}, ...
    'ctrl',  {{'kpHeading','kpDistance','arriveTol','headingTolDeg'}}, ...
    'mqtt',  {{'broker','port','topicCmd','topicTelemetry','topicCamPose'}});

groups = fieldnames(spec);
for gi = 1:numel(groups)
    g = groups{gi};
    assert(isfield(P, g), 'missing group P.%s', g);
    for fi = 1:numel(spec.(g))
        f = spec.(g){fi};
        assert(isfield(P.(g), f), 'missing field P.%s.%s', g, f);
    end
end

assert(abs(P.bot.maxWheelSpeed - 300/60*pi*0.034) < 1e-9, 'maxWheelSpeed wrong');
assert(P.bot.count == 3, 'bot count');
% link / camera consistency (TOPICS.md "Timing", aruco_node.py)
assert(abs(P.bot.trackEff - P.bot.track*P.bot.trackScale) < 1e-12, 'trackEff = track * trackScale');
assert(1/P.link.repeatHz < P.link.watchdog/2, 'bridge repeats must come at least twice per watchdog period');
assert(P.link.hostTimeout > 0.6 && P.link.hostTimeout > P.link.heartbeat*3, 'hostTimeout must exceed the longest brain stall (0.6 s)');
assert(P.cam.maxAge > P.cam.latency + P.link.netDelay, 'camera maxAge below the nominal lag');
assert(isequal(P.cam.robotIds, 1:P.bot.count), 'robot marker id = robot number');
assert(size(P.cam.cornerXY, 1) == numel(P.cam.cornerIds) && numel(P.cam.cornerIds) == 4, 'four corner markers');
assert(P.cam.markerSize + 2*P.cam.quietZone <= P.bot.chassisWidth, 'robot marker label wider than the robot');
assert(abs(P.cam.markerOffset - (P.bot.chassisLength/2 - P.bot.axleToBack)) < 1e-12, 'marker centred on the robot top');
xy = P.cam.cornerXY;
assert(all(xy(:,1) < 0 | xy(:,1) > P.arena.w | xy(:,2) < -P.arena.apron | xy(:,2) > P.arena.h), 'corner markers inside the driven area');
assert(P.arena.w == 1.0 && P.arena.h == 1.0, 'arena must be 1 x 1 m');
assert(abs(P.scale.arenaPerSite*P.scale.sitePerArena - 1) < 1e-12 && abs(P.scale.arenaPerSite - 0.005) < 1e-12, 'scale');
assert(P.truck.length == P.bot.chassisLength && P.truck.width == P.bot.chassisWidth && P.truck.height == 0.065, 'truck visual must match robot footprint');

% pile_geometry: V = pi r^2 h / 3
[r, h] = pile_geometry(1, 37);
assert(abs(pi*r^2*h/3 - 1) < 1e-9, 'cone volume mismatch');
assert(abs(h - r*tand(37)) < 1e-12, 'height mismatch');

% vectorised
[rv, hv] = pile_geometry([0 0.5 1 2], 37);
assert(isequal(size(rv), [1 4]) && isequal(size(hv), [1 4]), 'vector size');
assert(all(abs(pi*rv.^2.*hv/3 - [0 0.5 1 2]) < 1e-9), 'vector volume');

% invalid input must error
badCalls = {@() pile_geometry(-1, 37), @() pile_geometry(1, 0), ...
            @() pile_geometry(1, 90), @() pile_geometry('a', 37), ...
            @() pile_geometry(NaN, 37)};
for k = 1:numel(badCalls)
    threw = false;
    try, badCalls{k}(); catch, threw = true; end
    assert(threw, 'invalid input %d did not error', k);
end

disp('test_params PASSED');
