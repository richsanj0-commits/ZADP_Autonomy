function s = pose_fusion(s, z, P, tz)
%POSE_FUSION  Between fixes s.est follows the encoder increments (done in bot_step);
%   a camera fix z (non-empty), taken at time tz (default now), pulls s.est toward it with gain
%   P.cam.fusionGain (heading wrapped).
%   Latency compensation: the fix shows the pose at tz, not now. The odometry motion since tz
%   (from s.hist, as a rigid motion in the robot frame) is applied to z first, so the fix is
%   compared with the estimate at the SAME instant. Without it the estimate lags the truth by
%   about v*latency (10 mm at 0.2 m/s with 50 ms), more than the pile clearance.
if isempty(z), return; end
z = z(:)';
if nargin >= 4 && tz < s.t - 1e-12
    o0 = hist_at(s.hist, tz, 5:7); o1 = s.odom;
    c = cos(o0(3)); sn = sin(o0(3)); d = o1(1:2) - o0(1:2);
    rel = [c*d(1) + sn*d(2), -sn*d(1) + c*d(2)];            % odometry motion since tz, robot frame at tz
    dth = atan2(sin(o1(3) - o0(3)), cos(o1(3) - o0(3)));
    cz = cos(z(3)); sz = sin(z(3));
    z = [z(1) + cz*rel(1) - sz*rel(2), z(2) + sz*rel(1) + cz*rel(2), z(3) + dth];
end
e = z - s.est;
e(3) = atan2(sin(e(3)), cos(e(3)));
s.est = s.est + P.cam.fusionGain*e;
s.est(3) = atan2(sin(s.est(3)), cos(s.est(3)));
end
