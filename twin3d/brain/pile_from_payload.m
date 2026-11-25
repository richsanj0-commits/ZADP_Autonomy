function [r, h, V, A] = pile_from_payload(payloadTonnes, P)
%PILE_FROM_PAYLOAD  Cone pile of a dumped payload, exactly as the planner's calc(t) does it
%   (zadp_game.html calc(t); material numbers in P.pile, copied from MATERIALS).
%   [r, h, V, A] = pile_from_payload(payloadTonnes, P)
%   SITE scale (metres, m^3):  V = payload / rho,  r = cbrt(3 V / (pi tan(repose))),  h = r tan(repose)
%   so V = pi r^2 h / 3. No shape factor. payloadTonnes may be an array.
%   A = same pile at ARENA scale: A.r, A.h (m, x P.scale.arenaPerSite), A.V (m^3, x arenaPerSite^3).
V = payloadTonnes ./ P.pile.density;
[r, h] = pile_geometry(V, P.pile.reposeDeg);
s = P.scale.arenaPerSite;
A = struct('r', r*s, 'h', h*s, 'V', V*s^3);
end
