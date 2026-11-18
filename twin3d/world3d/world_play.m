function stats = world_play(log, plan, P, opts)
%WORLD_PLAY  Replay a brain log (brain_run) in the 3D world: trucks drive, the bed tips while dumping, the pile
%   grows during the dump and then stays; HUD shows sim time, site time, trucks dispatched, piles placed,
%   coverage and collisions.
%   stats = world_play(log, plan, P, opts)
%   opts.speed     sim seconds per video second (default 20)
%   opts.fps       video frame rate (default 30)
%   opts.tStart / opts.tEnd   window of the log in sim seconds (default whole log)
%   opts.view      'orbit' (default) | 'top' | 'gate' | 'follow'; opts.followBot (1..3), opts.orbitRate (deg of
%                  azimuth per video second, default 0)
%   opts.file      write a video: MPEG-4 (VideoWriter 'MPEG-4'); if that profile is unavailable an animated GIF
%                  (a .gif next to it). Hidden figure when a file is written.
%   opts.visible   'on' | 'off' (default 'on' without a file, 'off' with one); opts.size ([1280 720] px)
%   opts.quality  MPEG-4 quality 0..100 (default 50; the size of a 30 s clip is mostly set by it)
%   opts.maxFrames stop after this many frames (default all); opts.fig reuse a figure
%   stats: frames, wall (s), fps (frames per wall second), videoSeconds, file, bytes, fig (open figure when no
%   file was written, else []), world (the handle struct of world_build).
%   What the log supplies: pose (N-by-3-by-nb) and truck (N-by-nb) every logEvery seconds (poses are interpolated),
%   events (assigned), and the pile records (position, size, time of the pile; the dump ran for P.truck.dumpTime
%   before it: bed angle and growth follow dump_profile).
if nargin < 4, opts = struct(); end
def = struct('speed', 20, 'fps', 30, 'tStart', 0, 'tEnd', log.tEnd, 'view', 'orbit', 'followBot', 1, 'orbitRate', 0, ...
    'file', '', 'visible', '', 'size', [1280 720], 'maxFrames', Inf, 'fig', [], 'quality', 50);
f = fieldnames(def);
for i = 1:numel(f), if ~isfield(opts, f{i}), opts.(f{i}) = def.(f{i}); end, end
if isempty(opts.visible), opts.visible = 'on'; if ~isempty(opts.file), opts.visible = 'off'; end, end
nb = size(log.pose, 3); dumpT = P.truck.dumpTime;

% ---- precompute from the log
pr = log.pileRec; n = numel(pr);
pt = [pr.t]; pbot = [pr.bot];
S = struct('c', cell(1, n), 'r', 0, 'h', 0, 'frac', 0, 'key', 0);
for k = 1:n, S(k) = struct('c', log.piles(k,:), 'r', pr(k).rArena, 'h', pr(k).hArena, 'frac', 0, 'key', pr(k).spot); end
cov = coverage_curve(log.piles, [pr.rArena], plan);
ev = log.events; tAssigned = [ev(strcmp({ev.type}, 'assigned')).t];
m = log.mon; collisions = m.rrOverlapEvents + m.pileHitEvents + m.boundEvents;
nSpots = numel(log.spots);

% ---- scene and output
h = world_build(plan, P, struct('visible', opts.visible, 'size', opts.size, 'fig', opts.fig));
if ~strcmp(opts.visible, 'on'), fitFrame(h.fig, opts.size); end     % hidden: frames are exactly opts.size pixels
vw = []; gifFile = '';
if ~isempty(opts.file)
    try
        vw = VideoWriter(opts.file, 'MPEG-4'); vw.FrameRate = opts.fps; vw.Quality = opts.quality; open(vw);
    catch err
        warning('world_play:video', 'MPEG-4 unavailable (%s): writing an animated GIF instead', err.message);
        vw = []; gifFile = [regexprep(opts.file, '\.[^.\\/]*$', ''), '.gif'];
    end
end
dtv = opts.speed/opts.fps;
nF = min(opts.maxFrames, floor((opts.tEnd - opts.tStart)/dtv + 1e-9) + 1);
tic0 = tic; nWritten = 0;
for fi = 1:nF
    t = opts.tStart + (fi - 1)*dtv;
    % poses: linear between the log samples, heading along the short way
    i0 = find(log.t <= t, 1, 'last'); if isempty(i0), i0 = 1; end
    i1 = min(i0 + 1, numel(log.t));
    a = 0; if i1 > i0, a = (t - log.t(i0))/(log.t(i1) - log.t(i0)); end
    p0 = reshape(log.pose(i0,:,:), 3, nb)'; p1 = reshape(log.pose(i1,:,:), 3, nb)';
    pose = p0 + a*(p1 - p0);
    pose(:,3) = p0(:,3) + a*atan2(sin(p1(:,3) - p0(:,3)), cos(p1(:,3) - p0(:,3)));
    % dumps in progress -> bed angle, pile growth; finished piles stay
    bed = zeros(1, nb); fr = zeros(1, n);
    fr(pt <= t) = 1;
    act = find(pt > t & pt - dumpT <= t);
    if ~isempty(act)
        [bd, pf] = dump_profile((t - (pt(act) - dumpT))/dumpT, P);
        fr(act) = pf;
        for q = 1:numel(act), bed(pbot(act(q))) = max(bed(pbot(act(q))), bd(q)); end
    end
    nStarted = sum(pt - dumpT <= t);
    for k = 1:nStarted, S(k).frac = fr(k); end
    truck = double(log.truck(i0,:));
    world_sync(h, struct('pose', pose, 'bed', bed, 'truck', truck, 'piles', S(1:nStarted)));
    nDone = sum(pt <= t);
    world_hud(h, t, sum(tAssigned <= t), nDone, nSpots, 100*cov(nDone + 1), collisions, ...
        sprintf('replay x%g', opts.speed));
    switch lower(opts.view)
        case 'orbit', world_view(h, 'orbit', opts.orbitRate*(t - opts.tStart)/opts.speed);
        case 'follow', world_view(h, 'follow', opts.followBot);
        otherwise, if fi == 1, world_view(h, opts.view); end
    end
    if isempty(opts.file)
        drawnow limitrate;
        pause(max(0, (fi - 1)/opts.fps - toc(tic0)));          % real-time pacing for a visible replay
    else
        drawnow;
        fr0 = getframe(h.fig); nWritten = nWritten + 1;
        if ~isempty(vw)
            writeVideo(vw, fr0);
        else
            writeGif(gifFile, fr0.cdata, nWritten == 1, 1/opts.fps);
        end
    end
end
wall = toc(tic0);
file = opts.file; bytes = 0;
if ~isempty(vw), close(vw); end
if ~isempty(gifFile), file = gifFile; end
if ~isempty(file) && isfile(file), d = dir(file); bytes = d.bytes; end
stats = struct('frames', nF, 'wall', wall, 'fps', nF/wall, 'videoSeconds', nF/opts.fps, 'file', file, 'bytes', bytes, ...
    'fig', h.fig, 'world', h);
if isempty(opts.fig) && ~isempty(opts.file) && isgraphics(h.fig), close(h.fig); stats.fig = []; end
end

function fitFrame(fig, sz)
% make getframe return exactly sz pixels (display scaling changes the figure pixel size)
set(fig, 'Position', [60 60 sz]);
fr = getframe(fig);
r = size(fr.cdata, 2)/sz(1);
if abs(r - 1) > 1e-6
    set(fig, 'Position', [60 60 round(sz/r)]);
end
end

function writeGif(file, rgb, first, delay)
% animated GIF without toolboxes: fixed 6x6x6 colour cube palette
idx = uint8(36*round(double(rgb(:,:,1))/51) + 6*round(double(rgb(:,:,2))/51) + round(double(rgb(:,:,3))/51));
cm = zeros(216, 3);
q = 0; for ri = 0:5, for gi = 0:5, for bi = 0:5, q = q + 1; cm(q,:) = [ri gi bi]/5; end, end, end
if first
    imwrite(idx, cm, file, 'gif', 'LoopCount', Inf, 'DelayTime', delay);
else
    imwrite(idx, cm, file, 'gif', 'WriteMode', 'append', 'DelayTime', delay);
end
end
