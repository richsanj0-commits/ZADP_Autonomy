function [bedDeg, pileFrac] = dump_profile(f, P)
%DUMP_PROFILE  Bed angle and visible pile fraction while the dump runs. f = fraction of P.truck.dumpTime elapsed.
%   The bed rises over the first 25 % of the dump, stays up, and lowers over the last 25 %. The material runs out
%   between 20 % and 70 % (pileFrac 0 -> 1, see pile_growth); f <= 0 means no dump in progress (bed down, no
%   pile), f >= 1 a finished dump (bed down, full pile). Arrays allowed.
bedDeg = zeros(size(f)); pileFrac = zeros(size(f));
in = f > 0 & f < 1;
g = f(in);
up = min(g/0.25, 1) .* min((1 - g)/0.25, 1);
bedDeg(in) = P.truck.bedTiltMaxDeg * (up.^2 .* (3 - 2*up));          % smoothstep
pileFrac(in) = min(max((g - 0.2)/0.5, 0), 1);
pileFrac(f >= 1) = 1;
end
