function [r, h] = pile_geometry(V, reposeDeg)
%PILE_GEOMETRY  Radius and height of a conical pile from its volume.
%   [r, h] = pile_geometry(V, reposeDeg)
%   V         volume in m^3 (real, finite, >= 0), scalar or array
%   reposeDeg angle of repose in degrees (0 < angle < 90), scalar or array
%   Inputs of different sizes are combined by implicit expansion.
%
%   r = (3*V/(pi*tand(reposeDeg)))^(1/3),  h = r*tand(reposeDeg)
%   so that V = pi*r^2*h/3.

arguments
    V         {mustBeNumeric, mustBeReal, mustBeFinite, mustBeNonnegative}
    reposeDeg {mustBeNumeric, mustBeReal, mustBeFinite, mustBePositive}
end
if any(reposeDeg(:) >= 90)
    error('pile_geometry:badAngle', 'reposeDeg must be less than 90.');
end

t = tand(reposeDeg);
r = (3 .* V ./ (pi .* t)) .^ (1/3);
h = r .* t;
end
