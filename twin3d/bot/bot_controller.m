function [cmd, arrived] = bot_controller(pose, target, P)
%BOT_CONTROLLER  Go-to-point. pose=[x y th], target=[x y]. cmd=[vL vR] m/s.
%   Heading error > rotateThreshDeg: rotate in place (w = kpHeading*e, wheels limited to cruise speed).
%   Else drive: v = min(cruise, kpDistance*d, sqrt(2*decelFrac*maxAccel*d)) * cos(e), w = kpHeading*e
%   (w faded out within 3 cm of the target where the bearing is ill-defined). Arrived when d < arriveTol.
dx = target(1)-pose(1); dy = target(2)-pose(2);
d = hypot(dx, dy);
if d < P.ctrl.arriveTol
    cmd = [0 0]; arrived = true; return
end
arrived = false;
e = atan2(dy, dx) - pose(3);
e = atan2(sin(e), cos(e));
b = P.bot.track;
if abs(e) > deg2rad(P.ctrl.rotateThreshDeg)
    wmax = P.bot.cruiseSpeed/(b/2);
    w = min(max(P.ctrl.kpHeading*e, -wmax), wmax);
    v = 0;
else
    v = min([P.bot.cruiseSpeed, P.ctrl.kpDistance*d, sqrt(2*P.ctrl.decelFrac*P.bot.maxAccel*d)])*cos(e);
    w = P.ctrl.kpHeading*e*min(1, d/0.03);
end
cmd = [v - w*b/2, v + w*b/2];
end
