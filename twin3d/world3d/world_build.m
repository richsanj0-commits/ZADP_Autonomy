function h = world_build(plan, P, opts)
%WORLD_BUILD  The 3D scene of the dump site (base MATLAB graphics: patch, surf, hgtransform, lights).
%   h = world_build(plan, P, opts)   opts.visible ('off' default), opts.size ([1280 720] px),
%   opts.nTrucks (default P.bot.count), opts.fig (draw into this figure instead of a new one).
%   Arena metres, x east, y north, z up; the site is 1 x 1 m = 200 x 200 m (1:200), the apron lies at y < 0.
%   Draws: ground (dirt), the apron slab with parking slots P1..P3 (from P.brain.parking), the site as a slightly
%   raised pad with a low berm around the boundary, cut open at the gate, zones Z1 / Z2 as tinted floor patches,
%   the branch roads, the two-lane spine with IN (blue) and OUT (red) centre lines and arrows, the gate ramp and
%   gate posts, the trucks (truck_build), a HUD text area (second, flat axes on top), sun + head light.
%   h fields: fig, ax, hudAx, hud (struct of texts), truck (1-by-nTrucks), parkMark / parkLabel (1-by-3),
%   zoneLabel, light, headlight, colors, P, plan, nTrucks. Move things with world_sync, the camera with world_view.
if nargin < 3, opts = struct(); end
if ~isfield(opts, 'visible'), opts.visible = 'off'; end
if ~isfield(opts, 'size'), opts.size = [1280 720]; end
nT = P.bot.count; if isfield(opts, 'nTrucks'), nT = opts.nTrucks; end
sky = [0.80 0.88 0.95];
if isfield(opts, 'fig') && ~isempty(opts.fig)
    fig = opts.fig; clf(fig);
else
    fig = figure('Name', 'ZADP twin 3D', 'NumberTitle', 'off', 'Visible', opts.visible, 'Color', sky, ...
        'Position', [60 60 opts.size], 'MenuBar', 'none', 'ToolBar', 'none');
end
try, theme(fig, 'light'); catch, end
set(fig, 'Color', sky);
ax = axes('Parent', fig, 'Position', [0 0 1 1], 'Color', sky);
hold(ax, 'on');
cG = [0.56 0.49 0.39];      % ground dirt
cPad = [0.76 0.72 0.64];    % raised site pad
cBerm = [0.47 0.41 0.34];
cRoad = [0.36 0.37 0.40];
cBranch = [0.44 0.45 0.47];
cApron = [0.50 0.52 0.56];
zoneCol = [0.60 0.74 0.70; 0.78 0.68 0.50];
zPad = 0.004; zZone = 0.0046; zRoad = 0.0052; zLine = 0.0060;

% ---- ground
G = [-1.5 -1.6; 2.5 -1.6; 2.5 2.4; -1.5 2.4];
flat_poly(ax, G, -0.0005, cG);

% ---- apron slab (raised 1.5 mm) with parking slots
A = [-0.02 -P.arena.apron-0.02; 1.02 -P.arena.apron-0.02; 1.02 0.0; -0.02 0.0];
solid_box(ax, [A(1,1) A(2,1)], [A(1,2) A(3,2)], [0 0.0015], cApron);
bk = P.brain.parking;
slotL = 0.125; slotW = 0.105;
h.parkMark = gobjects(1, size(bk, 1)); h.parkLabel = gobjects(1, size(bk, 1));
for k = 1:size(bk, 1)
    c = bk(k, 1:2); a = bk(k, 3);
    ex = [cos(a) sin(a)]*slotL/2; ey = [-sin(a) cos(a)]*slotW/2;
    q = [c - ex - ey; c + ex - ey; c + ex + ey; c - ex + ey];
    h.parkMark(k) = patch('Parent', ax, 'Vertices', [q, 0.0022*ones(4,1)], 'Faces', 1:4, 'FaceColor', cApron*1.12, ...
        'EdgeColor', [1 1 1], 'LineWidth', 2, 'FaceLighting', 'flat', 'AmbientStrength', 0.6);
    h.parkLabel(k) = text('Parent', ax, 'Position', [c(1), c(2) - 0.075, 0.004], 'String', sprintf('P%d', k), ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 10, 'Color', [1 1 1], 'Clipping', 'off');
end
text('Parent', ax, 'Position', [0.12 -0.225 0.004], 'String', 'APRON / PARKING', 'FontSize', 9, 'FontWeight', 'bold', ...
    'Color', [0.95 0.95 0.95], 'Clipping', 'off');

% ---- site: raised pad (boundary grown by the berm foot) with a low berm, open at the gate
B = plan.boundary;
padPoly = offset_poly(B, 0.006);
box_poly(ax, padPoly, 0, zPad, cPad);
gx = [min(plan.spine.poly(:,1)) max(plan.spine.poly(:,1))];       % gate = the spine width at the entry
gy = plan.entry.pt(2);
berm_build(ax, B, gx, gy, cBerm);
% gate ramp from the apron (1.5 mm) up to the pad (4 mm), and gate posts
ramp = [gx(1) 0 0.0015; gx(2) 0 0.0015; gx(2) gy zPad+0.0012; gx(1) gy zPad+0.0012];
patch('Parent', ax, 'Vertices', ramp, 'Faces', 1:4, 'FaceColor', cRoad, 'EdgeColor', 'none', 'FaceLighting', 'flat', ...
    'AmbientStrength', 0.6);
for xg = gx
    solid_box(ax, xg + [-0.008 0.008], gy + [-0.008 0.008], [0 0.040], [0.95 0.75 0.10]);
    solid_box(ax, xg + [-0.0085 0.0085], gy + [-0.0085 0.0085], [0.030 0.034], [0.15 0.15 0.15]);
end
text('Parent', ax, 'Position', [mean(gx), gy, 0.050], 'String', 'GATE', 'HorizontalAlignment', 'center', ...
    'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.35 0.1 0.05], 'Clipping', 'off');

% ---- zones and branch roads (tinted floor patches)
h.zoneLabel = gobjects(1, numel(plan.zones));
for i = 1:numel(plan.zones)
    z = plan.zones(i); col = zoneCol(mod(i-1, size(zoneCol, 1)) + 1, :);
    flat_poly(ax, z.poly, zZone, col);
    c = mean(z.poly, 1);
    h.zoneLabel(i) = text('Parent', ax, 'Position', [c(1), max(z.poly(:,2)) + 0.032, 0.030], 'String', sprintf('Z%d', z.n), ...
        'HorizontalAlignment', 'center', 'FontSize', 16, 'FontWeight', 'bold', 'Color', col*0.45, 'Clipping', 'off');
    for k = 1:numel(z.branch)
        flat_poly(ax, z.branch{k}, zRoad, cBranch);
    end
end
% ---- spine (two lanes) with IN / OUT centre lines and arrows
flat_poly(ax, plan.spine.poly, zRoad, cRoad);
lanes = {plan.spine.inLine, [0.15 0.38 0.90], 'IN'; plan.spine.outLine, [0.88 0.20 0.15], 'OUT'};
for k = 1:2
    ln = lanes{k,1}; col = lanes{k,2};
    d = ln(end,:) - ln(1,:); L = norm(d); d = d/L; nrm = [-d(2) d(1)];
    q = [ln(1,:) - 0.002*nrm; ln(end,:) - 0.002*nrm; ln(end,:) + 0.002*nrm; ln(1,:) + 0.002*nrm];
    flat_poly(ax, q, zLine, col);
    for s = 0.06:0.12:L - 0.04
        p0 = ln(1,:) + d*s;
        tri = [p0 + d*0.022; p0 - d*0.012 + nrm*0.014; p0 - d*0.012 - nrm*0.014];
        flat_poly(ax, tri, zLine + 0.0002, col);
    end
    text('Parent', ax, 'Position', [ln(1,1), plan.spine.poly(1,2) - 0.016, 0.030], ...
        'String', lanes{k,3}, 'HorizontalAlignment', 'center', 'FontSize', 9, 'FontWeight', 'bold', 'Color', col, ...
        'Clipping', 'off');
end

% ---- trucks
h.colors = [0.85 0.33 0.10; 0.00 0.45 0.74; 0.47 0.67 0.19; 0.60 0.20 0.65; 0.9 0.8 0.1];
for i = 1:nT
    h.truck(i) = truck_build(ax, P, h.colors(mod(i-1, size(h.colors,1)) + 1, :));
    truck_pose(h.truck(i), [bk(min(i, size(bk,1)), 1:2), pi], 0, 0);
end

% ---- axes, lights, material
axis(ax, 'equal'); axis(ax, 'off');
set(ax, 'XLim', [G(1,1) G(2,1)], 'YLim', [G(1,2) G(3,2)], 'ZLim', [-0.01 0.4], 'Projection', 'perspective', ...
    'DataAspectRatio', [1 1 1], 'Clipping', 'off', 'AmbientLightColor', [0.72 0.72 0.72], 'SortMethod', 'depth');
set(ax, 'Color', sky);
h.light = light(ax, 'Style', 'infinite', 'Position', [-0.6 -0.8 1.0], 'Color', [1.0 0.96 0.88]);     % sun
h.light(2) = light(ax, 'Style', 'infinite', 'Position', [0.9 0.2 0.5], 'Color', [0.30 0.33 0.38]);     % cool fill
h.headlight = camlight(ax, 'headlight'); set(h.headlight, 'Color', [0.35 0.35 0.35]);
lighting(ax, 'flat');        % hold stays on: low-level graphics only, so later piles never reset the axes

% ---- HUD (flat overlay axes)
hud = axes('Parent', fig, 'Position', [0 0 1 1], 'Visible', 'off', 'XLim', [0 1], 'YLim', [0 1], 'HitTest', 'off');
hold(hud, 'on');
fill(hud, [0.010 0.245 0.245 0.010], [0.655 0.655 0.985 0.985], [1 1 1], 'FaceAlpha', 0.80, 'EdgeColor', [0.7 0.72 0.76]);
h.hud.title = text(hud, 0.018, 0.950, 'ZADP dump site twin', 'FontUnits', 'normalized', 'FontSize', 0.034, ...
    'FontWeight', 'bold', 'Color', [0.1 0.12 0.16]);
h.hud.sub = text(hud, 0.018, 0.905, sprintf('3 haul trucks, 1 m = 200 m (1:%d)', P.scale.sitePerArena), ...
    'FontUnits', 'normalized', 'FontSize', 0.022, 'Color', [0.25 0.28 0.33]);
h.hud.stats = text(hud, 0.018, 0.775, '', 'FontUnits', 'normalized', 'FontSize', 0.026, 'Color', [0.1 0.12 0.16], ...
    'VerticalAlignment', 'middle', 'FontName', 'Consolas');
h.hud.note = text(hud, 0.99, 0.02, '', 'FontUnits', 'normalized', 'FontSize', 0.022, 'Color', [0.25 0.28 0.33], 'HorizontalAlignment', 'right');
hold(hud, 'off');
h.fig = fig; h.ax = ax; h.hudAx = hud; h.P = P; h.plan = plan; h.nTrucks = nT;
setappdata(fig, 'pileObj', struct('key', {}, 'hp', {}, 'frac', {}));
world_view(h, 'orbit');
end

% =====================================================================================
function p = flat_poly(ax, xy, z, col)
% horizontal polygon at height z
n = size(xy, 1);
p = patch('Parent', ax, 'Vertices', [xy(:,1:2), z*ones(n,1)], 'Faces', 1:n, 'FaceColor', col, 'EdgeColor', 'none', ...
    'FaceLighting', 'flat', 'AmbientStrength', 0.6, 'DiffuseStrength', 0.6, 'SpecularStrength', 0);
end

function box_poly(ax, xy, z0, z1, col)
% prism of a convex polygon between heights z0 and z1: top face + side walls
n = size(xy, 1);
flat_poly(ax, xy, z1, col);
for i = 1:n
    j = mod(i, n) + 1;
    V = [xy(i,:) z0; xy(j,:) z0; xy(j,:) z1; xy(i,:) z1];
    patch('Parent', ax, 'Vertices', V, 'Faces', 1:4, 'FaceColor', col*0.78, 'EdgeColor', 'none', 'FaceLighting', 'flat', ...
        'AmbientStrength', 0.55);
end
end

function q = offset_poly(B, d)
% convex polygon (counter-clockwise or not) grown outward by d, mitred corners
n = size(B, 1);
if sum(B(:,1).*B([2:end 1],2) - B([2:end 1],1).*B(:,2)) < 0, B = flipud(B); end
q = zeros(n, 2);
for i = 1:n
    a = B(mod(i-2, n) + 1, :); b = B(i, :); c = B(mod(i, n) + 1, :);
    t1 = (b - a)/norm(b - a); t2 = (c - b)/norm(c - b);
    n1 = [t1(2) -t1(1)]; n2 = [t2(2) -t2(1)];             % outward normals (counter-clockwise polygon)
    m = n1 + n2; m = m/ (1 + n1*n2');                     % mitre: |m| d reaches both offset lines
    q(i,:) = b + d*m;
end
end

function berm_build(ax, B, gx, gy, col)
% low berm (trapezoid section, 14 mm high) just outside the boundary, from the gate's east edge round to its west edge
if sum(B(:,1).*B([2:end 1],2) - B([2:end 1],1).*B(:,2)) < 0, B = flipud(B); end
foot0 = 0.006; top0 = 0.010; top1 = 0.014; foot1 = 0.022; hgt = 0.014;
Q = {offset_poly(B, foot0), offset_poly(B, top0), offset_poly(B, top1), offset_poly(B, foot1)};
% ring starting at the gate's east edge: insert gate points on the bottom edge (the edge at y = gy)
n = size(B, 1);
e = find(abs(B(:,2) - gy) < 1e-9 & abs(B([2:end 1],2) - gy) < 1e-9, 1);     % edge e -> e+1 lies on the entry line
ring = cell(1, 4);
for k = 1:4
    q = Q{k}; i1 = e; i2 = mod(e, n) + 1;
    e1 = [gx(2), q(i1,2)]; e0 = [gx(1), q(i1,2)];        % points on that offset edge at the gate edges
    r = [e1; q(i2:end,:); q(1:i1,:); e0];                % east gate edge -> east ... -> west gate edge
    if i2 == 1, r = [e1; q(1:i1,:); e0]; end
    ring{k} = r;
end
m = size(ring{1}, 1);
z = [0 hgt hgt 0];
for s = 1:m-1
    for k = 1:3
        V = [ring{k}(s,:) z(k); ring{k}(s+1,:) z(k); ring{k+1}(s+1,:) z(k+1); ring{k+1}(s,:) z(k+1)];
        patch('Parent', ax, 'Vertices', V, 'Faces', 1:4, 'FaceColor', col*(0.9 + 0.1*(k==2)), 'EdgeColor', 'none', ...
            'FaceLighting', 'flat', 'AmbientStrength', 0.55);
    end
end
end
