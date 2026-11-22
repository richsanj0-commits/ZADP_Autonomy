function s = bot_step(s, cmd, P, dt)
%BOT_STEP  Advance the robot by dt. cmd = [vLcmd vRcmd] wheel speeds, m/s.
%   Motor: clip to +-maxWheelSpeed, first-order lag (motorTimeConstant), accel limit (maxAccel).
%   Encoders count wheel ROTATION exactly: enc = floor(wheelAngle/(2*pi)*CPR).
%   Slip: ground distance = rotation distance * k, k per wheel. Independent white noise per step
%   would average out (error ~ sqrt(distance)), so k is a stationary Ornstein-Uhlenbeck process in
%   distance travelled: mean 1, std slipStd, correlation length P.bot.slipCorrLength. Each wheel then
%   has a slowly varying scale error. The left/right difference integrates into heading, so with the
%   camera off the error grows faster than linearly: measured (slipStd 0.01, corr. 0.2 m, 150 runs)
%   about 1.7 % position and 1.7 deg heading over 0.2 m, 6 % / 6 deg over 1 m, 13 % / 13 deg over
%   4 m, independent of dt.
%   The true pose uses the exact arc; s.odom and s.est advance from the encoder count deltas.
%   Wheel travel per step is trapezoidal ((v_old + v_new)/2*dt), so distance while accelerating does
%   not depend on dt. s.hist keeps the last few [t, true pose, odom] rows for the camera latency.
vmax = P.bot.maxWheelSpeed;
cmd = min(max(cmd(:)', -vmax), vmax);
v = s.v + (cmd - s.v)*(1 - exp(-dt/P.bot.motorTimeConstant));
dv = min(max(v - s.v, -P.bot.maxAccel*dt), P.bot.maxAccel*dt);
v = s.v + dv;
dRot = (s.v + v)/2*dt;                         % rotation distance per wheel (trapezoidal: dt-independent)
s.wheelAngle = s.wheelAngle + dRot/(P.bot.wheelDiameter/2);
encNew = floor(s.wheelAngle/(2*pi)*P.bot.encoderCPR);
dEnc = encNew - s.enc;
a = exp(-abs(dRot)/P.bot.slipCorrLength);
s.slip = 1 + (s.slip-1).*a + P.bot.slipStd*sqrt(1-a.^2).*randn(s.rs, 1, 2);
dG = dRot.*s.slip;
% exact arc for the true pose
ds = (dG(1)+dG(2))/2; dth = (dG(2)-dG(1))/P.bot.trackEff; th = s.pose(3);   % effective track (trackScale)
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
% pose history for the camera latency (cam_measure / pose_fusion): enough rows to reach latency back
nKeep = ceil(P.cam.latency/dt) + 3;
s.hist = [s.hist(max(1, end - nKeep + 2):end, :); s.t, s.pose, s.odom];
end
