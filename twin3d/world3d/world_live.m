function [W, h] = world_live(plan, P, opts)
%WORLD_LIVE  Run the brain step by step and draw it live, so the twin can be watched while it computes.
%   [W, h] = world_live(plan, P, opts)
%   opts.maxTime  sim seconds to run (default 120; stops earlier when the brain is done)
%   opts.speed    sim seconds per wall second the display aims for (default 10); the brain is computed at
%                 P.brain.dt and the picture refreshed every speed/fps sim seconds (drawnow limitrate), paused
%                 to keep that pace when the computer is faster than that
%   opts.fps      refreshes per second (default 20); opts.seed (1); opts.view ('orbit'|'top'|'gate'|'follow');
%   opts.visible ('on'); opts.maxFrames (Inf); opts.orbitRate (deg of azimuth per wall second, 0)
%   The HUD collisions counter checks robot-robot and robot-pile overlaps at every refresh (brain_run checks every
%   step; use it for the safety statistics). Returns the final brain world W and the world handles h.
if nargin < 3, opts = struct(); end
def = struct('maxTime', 120, 'speed', 10, 'fps', 20, 'seed', 1, 'view', 'orbit', 'visible', 'on', 'maxFrames', Inf, ...
    'orbitRate', 0);
f = fieldnames(def);
for i = 1:numel(f), if ~isfield(opts, f{i}), opts.(f{i}) = def.(f{i}); end, end
rng(opts.seed);
W = brain_init(plan, P, opts.seed);
h = world_build(plan, P, struct('visible', opts.visible));
dt = P.brain.dt; nb = numel(W.bots);
stepsPerFrame = max(1, round(opts.speed/opts.fps/dt));
cov = 0; nPilesCov = 0; collisions = 0; hit = false(nb); hitP = false(1, nb);
nSpots = numel(W.spots);
tic0 = tic; frame = 0;
while W.t < opts.maxTime && ~W.done && frame < opts.maxFrames && isgraphics(h.fig)
    for q = 1:stepsPerFrame
        W = brain_step(W, P, dt);
        if W.done || W.t >= opts.maxTime, break; end
    end
    frame = frame + 1;
    s = brain_view_state(W, P);
    world_sync(h, s);
    if size(W.piles, 1) ~= nPilesCov, cov = 100*brain_coverage(W, plan); nPilesCov = size(W.piles, 1); end
    % collision monitor at refresh times
    for i = 1:nb
        for j = i+1:nb
            ov = rect_pair(s.pose(i,:), s.pose(j,:), P);
            collisions = collisions + (ov && ~hit(i,j)); hit(i,j) = ov;
        end
        if ~isempty(W.piles)
            ov = rect_circle_clear(s.pose(i,:), W.piles, W.pileR(:)', P) < 0;
            collisions = collisions + (ov && ~hitP(i)); hitP(i) = ov;
        end
    end
    world_hud(h, W.t, W.truckNo, size(W.piles, 1), nSpots, cov, collisions, sprintf('live, x%g', opts.speed));
    switch lower(opts.view)
        case 'orbit', world_view(h, 'orbit', opts.orbitRate*toc(tic0));
        case 'follow', world_view(h, 'follow', 1);
        otherwise, if frame == 1, world_view(h, opts.view); end
    end
    drawnow limitrate;
    pause(max(0, W.t/opts.speed - toc(tic0)));              % hold the display pace; no pause when the brain is slower
end
end
