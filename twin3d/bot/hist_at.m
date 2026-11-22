function p = hist_at(H, t, cols)
%HIST_AT  Pose [x y th] from the robot pose history H (rows [t, true x y th, odom x y th], oldest
%   first) at time t, linearly interpolated between the two rows around t (heading unwrapped);
%   clamped to the first/last row. cols = 2:4 (true pose) or 5:7 (odometry).
k = find(H(:,1) <= t + 1e-12, 1, 'last');
if isempty(k), p = H(1, cols); return; end
if k == size(H,1) || H(k+1,1) <= H(k,1), p = H(k, cols); return; end
f = (t - H(k,1))/(H(k+1,1) - H(k,1));
a = H(k, cols); b = H(k+1, cols);
b(3) = a(3) + atan2(sin(b(3) - a(3)), cos(b(3) - a(3)));
p = a + f*(b - a);
end
