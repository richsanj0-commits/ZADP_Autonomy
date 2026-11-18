function s = bot_step(s, cmd, P, dt)
%BOT_STEP  Advance the robot by dt. cmd = [vLcmd vRcmd] wheel speeds, m/s.
%   Motor: clip to +-maxWheelSpeed, first-order lag (motorTimeConstant), accel limit (maxAccel).
%   Encoders count wheel ROTATION exactly: enc = floor(wheelAngle/(2*pi)*CPR).
%   Slip: ground distance = rotation distance * k, k per wheel. Independent white noise per step
%   would average out (error ~ sqrt(distance)), so k is a stationary Ornstein-Uhlenbeck process in
%   distance travelled: mean 1, std slipStd, correlation length P.bot.slipCorrLength. Each wheel then
%   has a slowly varying scale error, so odometry drifts about slipStd * distance, independent of dt.
%   The true pose uses the exact arc; s.odom and s.est advance from the encoder count deltas.
vmax = P.bot.maxWheelSpeed;
cmd = min(max(cmd(:)', -vmax), vmax);
v = s.v + (cmd - s.v)*(1 - exp(-dt/P.bot.motorTimeConstant));
dv = min(max(v - s.v, -P.bot.maxAccel*dt), P.bot.maxAccel*dt);
v = s.v + dv;
dRot = v*dt;                                   % rotation distance per wheel
s.wheelAngle = s.wheelAngle + dRot/(P.bot.wheelDiameter/2);
encNew = floor(s.wheelAngle/(2*pi)*P.bot.encoderCPR);
dEnc = encNew - s.enc;
a = exp(-abs(dRot)/P.bot.slipCorrLength);
s.slip = 1 + (s.slip-1).*a + P.bot.slipStd*sqrt(1-a.^2).*randn(1,2);
dG = dRot.*s.slip;
% exact arc for the true pose
ds = (dG(1)+dG(2))/2; dth = (dG(2)-dG(1))/P.bot.track; th = s.pose(3);
if abs(dth) < 1e-9
    s.pose(1) = s.pose(1) + ds*cos(th);
    s.pose(2) = s.pose(2) + ds*sin(th);
else
    R = ds/dth;
    s.pose(1) = s.pose(1) + R*(sin(th+dth) - sin(th));
    s.pose(2) = s.pose(2) - R*(cos(th+dth) - cos(th));
end
s.pose(3) = th + dth;
s.v = v; s.enc = encNew;
s.odom = bot_odometry(s.odom, dEnc, P);
s.est  = bot_odometry(s.est,  dEnc, P);       % est follows odometry increments between camera fixes
s.t = s.t + dt;
end
