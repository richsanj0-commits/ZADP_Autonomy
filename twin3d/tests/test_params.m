% TEST_PARAMS  Checks twin_params and pile_geometry. Prints "test_params PASSED".
P = twin_params();

spec = struct( ...
    'bot',   {{'wheelDiameter','track','encoderCPR','chassisLength','chassisWidth', ...
               'axleToNose','axleToBack','motorNoLoadRpm','maxWheelSpeed','cruiseSpeed', ...
               'maxAccel','motorTimeConstant','slipStd','count'}}, ...
    'arena', {{'w','h'}}, ...
    'scale', {{'arenaPerSite','sitePerArena'}}, ...
    'cam',   {{'rateHz','posStd','headingStdDeg','latency'}}, ...
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
