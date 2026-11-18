function world_view(h, mode, k)
%WORLD_VIEW  Set the camera of a world from world_build.
%   world_view(h, 'top')        straight down over the whole site and apron
%   world_view(h, 'orbit', az)  default view from the south-west above the gate; az = extra azimuth in degrees
%                               (0 default) so a loop over az orbits the site
%   world_view(h, 'gate', k)    low view from the apron through the gate up the spine (k = height factor, 1)
%   world_view(h, 'follow', k)  chase view behind robot / truck k
%   Perspective projection; the head light follows the camera.
if nargin < 3, k = []; end
ax = h.ax;
set(ax, 'Projection', 'perspective', 'CameraUpVector', [0 0 1]);
tgt = [0.5 0.40 0.0];
switch lower(mode)
    case 'top'
        d = 2.55; pos = tgt + [0 0 d]; ang = 30;
        set(ax, 'CameraUpVector', [0 1 0]);
        tgt = [0.5 0.38 0];
        pos = tgt + [0 0 d];
    case 'orbit'
        if isempty(k), k = 0; end
        az = deg2rad(-28 + k); el = deg2rad(42); d = 2.05; ang = 29;
        tgt = [0.54 0.33 0];
        pos = tgt + d*[sin(az)*cos(el), -cos(az)*cos(el), sin(el)];
    case 'gate'
        if isempty(k), k = 1; end
        pos = [0.5 -0.34 0.16*k]; tgt = [0.5 0.30 0.00]; ang = 42;
    case 'follow'
        if isempty(k), k = 1; end
        M = get(h.truck(k).root, 'Matrix');
        p = M(1:2, 4)'; th = atan2(M(2,1), M(1,1)); dv = [cos(th) sin(th)];
        pos = [p - 0.34*dv, 0.19]; tgt = [p + 0.06*dv, 0.025]; ang = 38;
    otherwise
        error('world_view:mode', 'unknown view "%s" (top, orbit, gate, follow)', mode);
end
set(ax, 'CameraPosition', pos, 'CameraTarget', tgt, 'CameraViewAngle', ang);
if isfield(h, 'headlight') && isgraphics(h.headlight), camlight(h.headlight, 'headlight'); end
end
