% TEST_WORLD3D  Phase 6: the 3D world (hidden figures, nothing is shown).
%   Builds the world: 3 trucks, 3 parking markers P1..P3 at P.brain.parking, truck footprint 0.100 x 0.084 m with
%   the origin at the rear axle; truck_pose moves the truck and tips the bed (clamped to P.truck.bedTiltMaxDeg);
%   a pile cone has the requested radius and height within 1 % and grows with pile_growth; world_play renders 20
%   frames of a short brain log (a dump in progress: a pile appears); world_live runs a few frames; the Simulink
%   model zadp_twin runs 5 s with the 3D view updating (skipped with a message when Simulink cannot be loaded).
%   Prints "test_world3d PASSED".
here = fileparts(mfilename('fullpath'));
P = twin_params();
plan = load_plan(fullfile(here, '..', 'plan', 'plan_arena.json'));
figsBefore = numel(findall(0, 'Type', 'figure'));

% ---- world: handle counts
h = world_build(plan, P, struct('visible', 'off'));
assert(strcmp(get(h.fig, 'Visible'), 'off'), 'figure must be hidden');
assert(numel(h.truck) == 3, '3 trucks expected, got %d', numel(h.truck));
assert(numel(h.parkMark) == 3 && all(isgraphics(h.parkMark)), '3 parking markers expected');
assert(numel(h.parkLabel) == 3 && isequal(get(h.parkLabel, {'String'}), {'P1'; 'P2'; 'P3'}), 'parking labels P1..P3');
for k = 1:3
    v = get(h.parkMark(k), 'Vertices');
    assert(norm(mean(v(:,1:2), 1) - P.brain.parking(k,1:2)) < 1e-9, 'marker P%d not at P.brain.parking', k);
end
assert(all(arrayfun(@(t) isgraphics(t.root, 'hgtransform') && isgraphics(t.bed, 'hgtransform'), h.truck)), 'hgtransforms');
assert(numel(h.zoneLabel) == numel(plan.zones), 'zone labels');
for v = {'top', 'orbit', 'gate', 'follow'}, world_view(h, v{1}, 1); end   % every view works
world_view(h, 'orbit');

% ---- truck footprint (truck frame, bed down): 0.100 x 0.084 m, rear axle at the origin
ht = h.truck(1);
truck_pose(ht, [0 0 0], 0, 1);
V = truck_vertices(ht.root, eye(4));
assert(abs(min(V(:,1)) + P.bot.axleToBack) < 1e-3 && abs(max(V(:,1)) - P.bot.axleToNose) < 1e-3, ...
    'truck length: x in [%.4f %.4f]', min(V(:,1)), max(V(:,1)));
assert(abs(max(abs(V(:,2))) - P.truck.width/2) < 1e-3, 'truck width: |y| max %.4f', max(abs(V(:,2))));
assert(max(V(:,3)) <= P.truck.height + 1e-3 && max(V(:,3)) > 0.055, 'truck height %.4f', max(V(:,3)));
assert(strcmp(ht.label.String, 'T1') && strcmp(get(ht.label, 'Visible'), 'on'), 'label T1');

% ---- truck_pose moves and tips
truck_pose(ht, [0.3 0.4 pi/2], 0, 7);
M = get(ht.root, 'Matrix');
assert(abs(M(1,4) - 0.3) < 1e-12 && abs(M(2,4) - 0.4) < 1e-12 && abs(atan2(M(2,1), M(1,1)) - pi/2) < 1e-12, 'pose');
assert(strcmp(ht.label.String, 'T7'), 'label T7');
front0 = bed_front_z(ht);
truck_pose(ht, [0.3 0.4 pi/2], 30, 7);
front30 = bed_front_z(ht);
truck_pose(ht, [0.3 0.4 pi/2], 90, 7);                         % clamped to bedTiltMaxDeg
front90 = bed_front_z(ht);
truck_pose(ht, [0.3 0.4 pi/2], P.truck.bedTiltMaxDeg, 7);
frontMax = bed_front_z(ht);
assert(front30 > front0 + 0.01 && frontMax > front30, 'the bed front must rise with the angle');
assert(abs(front90 - frontMax) < 1e-12, 'bed angle must be clamped to %g deg', P.truck.bedTiltMaxDeg);
truck_pose(ht, [0.3 0.4 pi/2], 0, 0);
assert(strcmp(get(ht.label, 'Visible'), 'off'), 'truck number 0 hides the label');
truck_pose(ht, [0.3 0.4 pi/2], 0);                             % bed down again
assert(abs(bed_front_z(ht) - front0) < 1e-12, 'bed back down');

% ---- pile cone
r0 = 0.0353; h0 = 0.0266;
hp = pile_draw(h.ax, [0.3 0.6], r0, h0, P, 5);
X = get(hp.surf, 'XData') - 0.3; Y = get(hp.surf, 'YData') - 0.6; Z = get(hp.surf, 'ZData');
rr = max(hypot(X(:), Y(:))); hh = max(Z(:)) - min(Z(:));
assert(abs(rr/r0 - 1) < 0.01, 'pile radius %.5f vs %.5f', rr, r0);
assert(abs(hh/h0 - 1) < 0.01, 'pile height %.5f vs %.5f', hh, h0);
assert(all(arrayfun(@(a) norm(a) <= r0*(1 + 1e-9), hypot(X(:), Y(:)))), 'pile inside its own circle');
[rg, hg] = pile_growth(0.5, r0, h0); pile_update(hp, rg, hg);
X = get(hp.surf, 'XData') - 0.3; Y = get(hp.surf, 'YData') - 0.6; Z = get(hp.surf, 'ZData');
assert(abs(max(hypot(X(:), Y(:)))/rg - 1) < 0.01 && abs((max(Z(:)) - min(Z(:)))/hg - 1) < 0.01, 'grown pile size');
assert(any(Z(:) > min(Z(:))), 'pile has height');
close(h.fig);

% ---- world_play: 20 frames of a short log (dump of the first truck runs at about 14-18 s)
[~, log] = brain_run(plan, P, struct('seed', 1, 'maxTime', 25));
assert(size(log.truck, 1) == numel(log.t) && max(log.truck(:)) >= 1, 'log must hold the truck numbers');
st = world_play(log, plan, P, struct('speed', 30, 'fps', 100, 'tStart', 12, 'maxFrames', 20, 'visible', 'off', 'view', 'orbit'));
assert(st.frames == 20 && isgraphics(st.fig), 'world_play frames');
po = getappdata(st.fig, 'pileObj');
assert(numel(po) >= 1 && po(1).frac > 0, 'a pile must have appeared during the dump');
world_play(log, plan, P, struct('speed', 30, 'fps', 100, 'tStart', 12, 'maxFrames', 3, 'visible', 'off', 'fig', st.fig));
close(st.fig);
f = [tempname '.mp4'];
s2 = world_play(log, plan, P, struct('speed', 10, 'fps', 30, 'tStart', 12, 'maxFrames', 6, 'file', f));
assert(s2.bytes > 1000 && isfile(s2.file), 'video file');
delete(s2.file);

% ---- world_live: a few refreshes of the live run
[Wl, hl] = world_live(plan, P, struct('maxTime', 20, 'speed', 200, 'fps', 20, 'maxFrames', 4, 'visible', 'off'));
assert(Wl.t > 0 && isgraphics(hl.fig), 'world_live');
close(hl.fig);

% ---- Simulink: model runs 5 s with the 3D view updating
try
    assert(license('test', 'Simulink') == 1, 'no Simulink licence');
    build_twin_model(fullfile(here, '..', 'simulink'));
    simOK = true;
catch err
    simOK = false;
    fprintf('  test_world3d: SKIPPED the Simulink check, Simulink could not be loaded (%s)\n', err.message);
end
if simOK
    r = run_twin_model(5);
    assert(abs(r.simTime - 5) < 1e-6, 'model sim time %.3f', r.simTime);
    assert(r.renders >= 5, 'the 3D view must update (%d updates)', r.renders);
    assert(isfile(r.png), 'last picture');
end
assert(numel(findall(0, 'Type', 'figure')) <= figsBefore, 'all figures closed');
disp('test_world3d PASSED');

% ===================================================================================
function V = truck_vertices(node, M)
% all patch vertices below an hgtransform, in the frame of the top node, using the transforms below it
V = zeros(0, 3);
kids = get(node, 'Children');
for i = 1:numel(kids)
    c = kids(i);
    switch c.Type
        case 'patch'
            v = c.Vertices; v = (M(1:3,1:3)*v')' + M(1:3,4)';
            V = [V; v]; %#ok<AGROW>
        case 'hgtransform'
            V = [V; truck_vertices(c, M*c.Matrix)]; %#ok<AGROW>
    end
end
end

function z = bed_front_z(ht)
% height (truck frame) of the front-top corner of the dump body, from the bed transform
p = [0.050 - ht.bedBase(1,4); 0; 0.058 - ht.bedBase(3,4); 1];
q = get(ht.bed, 'Matrix')*p;                    % the bed matrix already contains the hinge translation
z = q(3);
end
