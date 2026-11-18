function ht = truck_build(ax, P, stripeColor)
%TRUCK_BUILD  Low-poly rigid-frame mining dump truck, built from boxes, prisms and wheels. No logos.
%   ht = truck_build(ax, P, stripeColor)
%   Footprint = the robot footprint P.truck.length x P.truck.width (0.100 x 0.084 m). Truck frame: origin at the
%   REAR-AXLE midpoint (the robot pose), x forward, y left, z up; the body spans x in [-axleToBack,
%   axleToNose] = [-0.024, 0.076]. Large rear wheels, smaller front wheels, cab on the front LEFT under the
%   canopy of the dump body, engine deck with grille on the front right, big tipping dump body hinged at the
%   rear.  Hierarchy: ht.root (hgtransform, moves the truck) > static parts + ht.bed (hgtransform, hinge at the
%   rear of the body, tips about the y axis) + ht.label (text above the truck).
%   ht.stripe = robot colour band on the canopy, ht.bedBase = hinge translation, ht.maxDeg = tilt limit.
if nargin < 3, stripeColor = [0.85 0.33 0.10]; end
yel = P.truck.bodyColor; bedc = P.truck.bedColor; tyre = P.truck.wheelColor;
dark = [0.22 0.22 0.25]; grey = [0.55 0.56 0.58]; glass = [0.20 0.30 0.40];
xb = -P.bot.axleToBack; xn = P.bot.axleToNose;                     % -0.024 .. 0.076
hw = P.truck.width/2;                                              % 0.042
root = hgtransform('Parent', ax);
% ---- chassis: frame rails, rear axle housing, front axle
solid_box(root, [xb+0.002 xn-0.002], [-0.028 0.028], [0.013 0.024], dark);
solid_box(root, [-0.004 0.004], [-0.036 0.036], [0.015 0.023], dark);
solid_box(root, [0.054 0.062], [-0.037 0.037], [0.011 0.019], dark);
% ---- wheels: big rear (radius 0.019, axle height = radius), smaller front
rz = 0.019; fz = 0.0155; wy = hw - 0.006; fy = hw - 0.005;
for s = [-1 1]
    solid_wheel(root, [0 s*wy rz], 0.019, 0.006, tyre, 14);
    solid_wheel(root, [0 s*(wy + 0.0003) rz], 0.0085, 0.0057, grey, 10);
    solid_wheel(root, [0.058 s*fy fz], 0.0155, 0.005, tyre, 12);
    solid_wheel(root, [0.058 s*(fy + 0.0005) fz], 0.007, 0.0045, grey, 10);
end
% ---- engine deck (front right): hood sloping down to the grille
solid_prism(root, [0.046 0.024; 0.076 0.024; 0.076 0.034; 0.062 0.040; 0.046 0.040], [-0.036 0.004], yel);
solid_box(root, [0.0755 0.076], [-0.031 -0.001], [0.026 0.033], dark);        % grille
solid_box(root, [0.0755 0.076], [-0.034 -0.031], [0.029 0.032], [1 1 0.85]);    % head lamps
solid_box(root, [0.0755 0.076], [-0.001 0.002], [0.029 0.032], [1 1 0.85]);
solid_box(root, [0.050 0.053], [-0.033 -0.030], [0.040 0.058], dark);          % exhaust stack
% ---- cab (front left)
solid_box(root, [0.046 0.070], [0.006 0.037], [0.024 0.054], yel);
solid_box(root, [0.0700 0.0705], [0.010 0.033], [0.037 0.050], glass);          % windscreen
solid_box(root, [0.050 0.066], [0.0370 0.0375], [0.037 0.050], glass);          % side window
solid_box(root, [0.044 0.072], [0.036 0.042], [0.022 0.026], dark);             % step / walkway
% ---- dump body (child hgtransform, geometry relative to the hinge at the rear)
H = [xb + 0.001, 0, 0.027];
bed = hgtransform('Parent', root);
ox = H(1); oz = H(3);
fl = [xb 0.026; 0.050 0.033; 0.050 0.036; xb 0.029];                 % floor slab, rising to the front
solid_prism(bed, fl - [ox oz], [-0.029 0.029], bedc);
sd = [xb 0.026; 0.050 0.033; 0.050 0.058; xb 0.042];                 % side plates, tall at the front
solid_prism(bed, sd - [ox oz], [0.025 0.029], bedc);
solid_prism(bed, sd - [ox oz], [-0.029 -0.025], bedc);
solid_box(bed, [0.046 0.050] - ox, [-0.029 0.029], [0.030 0.058] - oz, bedc);   % front wall
solid_box(bed, [-0.024 -0.021] - ox, [-0.029 0.029], [0.029 0.044] - oz, bedc*0.85);  % tail-gate
solid_box(bed, [0.046 0.074] - ox, [-0.037 0.037], [0.057 0.0605] - oz, bedc);  % canopy over the cab
st = solid_box(bed, [0.050 0.072] - ox, [-0.006 0.006], [0.0605 0.0620] - oz, stripeColor);   % robot colour band
solid_box(bed, [0.046 0.074] - ox, [-0.037 -0.0355], [0.052 0.057] - oz, bedc*0.8);     % canopy edge
solid_box(bed, [0.046 0.074] - ox, [0.0355 0.037], [0.052 0.057] - oz, bedc*0.8);
% hinge brackets and hoist ram (static)
solid_box(root, [xb xb+0.007], [-0.025 -0.019], [0.020 0.030], dark);
solid_box(root, [xb xb+0.007], [0.019 0.025], [0.020 0.030], dark);
solid_box(root, [0.026 0.029], [-0.003 0.003], [0.024 0.032], grey);
% ---- label above the truck and robot colour stripe on the cab side
lab = text('Parent', root, 'Position', [0.018 0 0.125], 'String', 'T0', 'HorizontalAlignment', 'center', ...
    'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.05 0.05 0.05], 'BackgroundColor', [1 1 1], ...
    'EdgeColor', stripeColor, 'LineWidth', 1.5, 'Margin', 1, 'Clipping', 'off');
solid_box(root, [0.044 0.072], [0.0375 0.0395], [0.028 0.035], stripeColor);     % stripe on the cab side
ht = struct('root', root, 'bed', bed, 'label', lab, 'stripe', st, 'bedBase', makehgtform('translate', H), ...
    'maxDeg', P.truck.bedTiltMaxDeg, 'color', stripeColor);
end
