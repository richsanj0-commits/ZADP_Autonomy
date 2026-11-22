function [cmd, arrived, rotating] = bot_controller(pose, target, P, rotating)
%BOT_CONTROLLER  Go-to-point. pose=[x y th], target=[x y]. cmd=[vL vR] m/s.
%   rotating (in/out, default false) = the robot is in the rotate-in-place mode (hysteresis state).
%   Far from the target (d >= P.ctrl.captureRadius):
%     rotate in place when the heading error exceeds rotateThreshDeg, and keep rotating until it is
%     below rotateExitDeg (hysteresis, no drive/rotate chatter at the threshold); w = kpHeading*e,
%     wheels limited to cruise speed.
%     Else drive: v = min(cruise, kpDistance*d, sqrt(2*decelFrac*maxAccel*d)) * cos(e), w = kpHeading*e.
%   Inside the capture radius the bearing is dominated by pose-estimate noise (mm), so the robot NEVER
%   rotates in place there: it drives with the same v (cos(e) < 0 backs up a little) and steers toward
%   the target along its own direction of travel (forward or reverse), with w faded out linearly to
%   zero at the target. Arrived when d < arriveTol.
if nargin < 4 || isempty(rotating), rotating = false; end
dx = target(1)-pose(1); dy = target(2)-pose(2);
d = hypot(dx, dy);
if d < P.ctrl.arriveTol
    cmd = [0 0]; arrived = true; rotating = false; return
end
arrived = false;
e = atan2(dy, dx) - pose(3);
e = atan2(sin(e), cos(e));
b = P.bot.trackEff;                 % effective track (trackScale)
capture = d < P.ctrl.captureRadius;
if capture
    rotating = false;
elseif rotating
    rotating = abs(e) > deg2rad(P.ctrl.rotateExitDeg);
else
    rotating = abs(e) > deg2rad(P.ctrl.rotateThreshDeg);
end
if rotating
    wmax = P.bot.cruiseSpeed/(b/2);
    w = min(max(P.ctrl.kpHeading*e, -wmax), wmax);
    v = 0;
else
    v = min([P.bot.cruiseSpeed, P.ctrl.kpDistance*d, sqrt(2*P.ctrl.decelFrac*P.bot.maxAccel*d)])*cos(e);
    eS = e;
    if cos(e) < 0, eS = atan2(sin(e + pi), cos(e + pi)); end    % backing up: steer the tail toward the target
    w = P.ctrl.kpHeading*eS*min(1, d/P.ctrl.captureRadius);
end
cmd = [v - w*b/2, v + w*b/2];
end
