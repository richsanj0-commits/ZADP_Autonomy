function demo_maps(mode)
%DEMO_MAPS  Full 3-robot fill (seed 1) on the four arena shapes in plan\: arena (square, one cut corner),
%   rect (wide rectangle), lshape (L, arm up the left side), hex (irregular hexagon). Same brain code and
%   parameters for all four. Per map it saves, in twin3d\plan\:
%     fill_<name>.png    the finished fill (brain_plot)
%     fill3d_<name>.png  a 3D still of the finished fill (world3d, hidden figure)
%   and at the end maps_results.md (one table row per map) and maps_results.mat (the numbers).
%   demo_maps()       runs in a MATLAB timer, one map per timer call, and returns at once, so no single
%                     call blocks for the whole run (about 1-3 min wall per map). Progress: maps_status.txt.
%   demo_maps('now')  runs everything in this call.
if nargin < 1, mode = 'timer'; end
here = fileparts(mfilename('fullpath'));
planDir = fullfile(fileparts(here), 'plan');
names = {'arena', 'rect', 'lshape', 'hex'};
writeStatus(planDir, 'started');
if strcmp(mode, 'now')
    R = [];
    for k = 1:numel(names), R = [R, runOne(planDir, names{k})]; end %#ok<AGROW>
    finish(planDir, R);
    return
end
old = timerfindall('Tag', 'demo_maps'); if ~isempty(old), stop(old); delete(old); end
t = timer('Tag', 'demo_maps', 'ExecutionMode', 'fixedSpacing', 'Period', 1, 'StartDelay', 1, ...
    'TasksToExecute', numel(names), 'UserData', struct('k', 0, 'R', []));
t.TimerFcn = @(tm, ~) step(tm, planDir, names);
t.StopFcn = @(tm, ~) stopped(tm, planDir);
t.ErrorFcn = @(tm, e) writeStatus(planDir, ['error: ' e.Data.message]);
start(t);
fprintf('demo_maps: started in a timer (%d maps); see %s\n', numel(names), fullfile(planDir, 'maps_status.txt'));
end

function step(tm, planDir, names)
u = tm.UserData; u.k = u.k + 1;
try
    r = runOne(planDir, names{u.k});
    u.R = [u.R, r];
catch err
    writeStatus(planDir, sprintf('error on %s: %s', names{u.k}, err.message));
end
tm.UserData = u;
end

function stopped(tm, planDir)
u = tm.UserData;
if ~isempty(u.R), finish(planDir, u.R); end
delete(tm);
end

function r = runOne(planDir, name)
P = twin_params();
file = fullfile(planDir, ['plan_' name '.json']);
plan = load_plan(file);
writeStatus(planDir, sprintf('running %s', name));
[W, log] = brain_run(plan, P, struct('seed', 1));
m = log.mon;
% zone lock from the log (Rule 2): never two robot positions inside one zone polygon
nb = size(log.pose, 3); zoneLock = 0;
for z = 1:numel(plan.zones)
    zp = plan.zones(z).poly; c = zeros(numel(log.t), 1);
    for i = 1:nb, c = c + inpolygon(log.pose(:,1,i), log.pose(:,2,i), zp(:,1), zp(:,2)); end
    zoneLock = zoneLock + nnz(c > 1);
end
B = plan.boundary;
u = find(~strcmp({W.spots.status}, 'placed'));
why = cell(1, numel(u));
for q = 1:numel(u), why{q} = sprintf('order %d %s: %s', W.spots(u(q)).order, W.spots(u(q)).kind, W.spots(u(q)).why); end
r = struct('name', name, 'siteArea', polyarea(B(:,1), B(:,2))/plan.scale^2, 'zones', numel(plan.zones), ...
    'spots', numel(W.spots), 'piles', size(W.piles, 1), 'coverage', 100*log.coverage, 'tonnes', log.tonnes, ...
    'simTime', log.tEnd, 'minGap', 1000*m.minRR, 'minPile', 1000*m.minPile, ...
    'viol', [m.rrOverlapEvents, m.pileHitEvents, m.boundEvents, m.ownerEvents, m.deadlocks, zoneLock], ...
    'maxInside', m.maxInside, 'endReason', log.endReason, 'wall', log.wall, 'unfilled', {why});
brain_plot(W, log, plan, fullfile(planDir, ['fill_' name '.png']));
% 3D still of the finished fill
h = world_build(plan, P, struct('visible', 'off'));
world_sync(h, brain_view_state(W, P));
world_hud(h, log.tEnd, log.trucks, size(W.piles, 1), numel(W.spots), 100*log.coverage, m.collisions, ['map ' name]);
world_view(h, 'orbit');
drawnow;
fr = getframe(h.fig);
imwrite(fr.cdata, fullfile(planDir, ['fill3d_' name '.png']));
close(h.fig);
fprintf('demo_maps %s: %d / %d piles, sim %.0f s, violations %s, wall %.0f s\n', name, r.piles, r.spots, r.simTime, mat2str(r.viol), r.wall);
writeStatus(planDir, sprintf('done %s (%d / %d)', name, r.piles, r.spots));
end

function finish(planDir, R)
save(fullfile(planDir, 'maps_results.mat'), 'R');
f = fopen(fullfile(planDir, 'maps_results.md'), 'w');
fprintf(f, '# Four arena shapes: full 3-robot fill, seed 1 (demo_maps)\n\n');
fprintf(f, 'Same brain code and parameters on every map. Violations = robot-robot / pile / off-site / section ownership / deadlock / zone lock events.\n');
fprintf(f, 'Images: `fill_<map>.png` (2D, brain_plot) and `fill3d_<map>.png` (3D still) in this folder.\n\n');
fprintf(f, '| Map | Site area (m^2) | Zones | Spots | Piles placed | Coverage %% | Tonnes | Sim time (s) | Min robot gap (mm) | Violations | Unfilled spots and reason |\n');
fprintf(f, '|---|---|---|---|---|---|---|---|---|---|---|\n');
for r = R
    if isempty(r.unfilled), uf = 'none'; else, uf = sprintf('%d: %s', numel(r.unfilled), strjoin(r.unfilled, '; ')); end
    fprintf(f, '| %s | %.0f | %d | %d | %d | %.1f | %.0f | %.0f | %.1f | %s | %s |\n', r.name, r.siteArea, r.zones, r.spots, ...
        r.piles, r.coverage, r.tonnes, r.simTime, r.minGap, strjoin(compose('%d', r.viol), ' / '), strrep(uf, '|', '/'));
end
fclose(f);
writeStatus(planDir, 'finished');
end

function writeStatus(planDir, s)
f = fopen(fullfile(planDir, 'maps_status.txt'), 'w'); fprintf(f, '%s %s\n', datestr(now, 'HH:MM:SS'), s); fclose(f); %#ok<TNOW1,DATST>
end
