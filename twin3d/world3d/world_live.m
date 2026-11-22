function [W, h] = world_live(plan, P, opts)
%WORLD_LIVE  Run the brain step by step and draw it live, so the twin can be watched while it computes.
%   [W, h] = world_live(plan, P, opts)
%   opts.maxTime  sim seconds to run (default 120; stops earlier when the brain is done)
%   opts.speed    sim seconds per wall second the display aims for (default 10); the brain is computed at
%                 P.brain.dt and the picture refreshed every speed/fps sim seconds (drawnow limitrate), paused
%                 to keep that pace when the computer is faster than that
%   opts.fps      refreshes per second (default 20); opts.seed (1); opts.view ('orbit'|'top'|'gate'|'follow');
%   opts.visible ('on'); opts.maxFrames (Inf); opts.orbitRate (deg of azimuth per wall second, 0)
%   The HUD collisions counter is brain_monitor's (robot-robot + robot-pile + off-site events), checked at every
%   brain step as in brain_run. The run depends on opts.seed only (the brain's own random stream; the global
%   stream is not touched). Closing the figure ends the run. Returns the final brain world W and the world handles h.
if nargin < 3, opts = struct(); end
def = struct('maxTime', 120, 'speed', 10, 'fps', 20, 'seed', 1, 'view', 'orbit', 'visible', 'on', 'maxFrames', Inf, ...
    'orbitRate', 0);
f = fieldnames(def);
for i = 1:numel(f), if ~isfield(opts, f{i}), opts.(f{i}) = def.(f{i}); end, end
W = brain_init(plan, P, opts.seed);
h = world_build(plan, P, struct('visible', opts.visible));
dt = P.brain.dt; nb = numel(W.bots);
stepsPerFrame = max(1, round(opts.speed/opts.fps/dt));
cov = 0; nPilesCov = 0; M = brain_monitor([], W, P);
nSpots = numel(W.spots);
tic0 = tic; frame = 0;
while W.t < opts.maxTime && ~W.done && ~W.deadlock && frame < opts.maxFrames && isgraphics(h.fig)
    for q = 1:stepsPerFrame
        W = brain_step(W, P, dt);
        M = brain_monitor(M, W, P);
        if W.done || W.deadlock || W.t >= opts.maxTime, break; end
    end
    if ~isgraphics(h.fig), break; end                       % closed while the brain was stepping
    frame = frame + 1;
    s = brain_view_state(W, P);
    world_sync(h, s);
    if size(W.piles, 1) ~= nPilesCov, cov = 100*brain_coverage(W, plan); nPilesCov = size(W.piles, 1); end
    world_hud(h, W.t, W.truckNo, size(W.piles, 1), nSpots, cov, brain_monitor_collisions(M), sprintf('live, x%g', opts.speed));
    switch lower(opts.view)
        case 'orbit', world_view(h, 'orbit', opts.orbitRate*toc(tic0));
        case 'follow', world_view(h, 'follow', 1);
        otherwise, if frame == 1, world_view(h, opts.view); end
    end
    drawnow limitrate;
    pause(max(0, W.t/opts.speed - toc(tic0)));              % hold the display pace; no pause when the brain is slower
end
end
