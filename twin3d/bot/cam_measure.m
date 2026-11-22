function [z, tz] = cam_measure(s, P)
%CAM_MEASURE  Overhead-camera pose fix taken P.cam.latency ago: z = true pose at time tz = s.t - latency
%   (linearly interpolated in the robot's short pose history s.hist) + Gaussian noise. tz is the image
%   stamp; pose_fusion uses it to move the fix forward to now with the odometry since tz (latency
%   compensation, as the firmware should do with its own odometry ring buffer).
%   Noise comes from s.rs (the robot's random stream, see bot_init).
tz = s.t - P.cam.latency;
pz = hist_at(s.hist, tz, 2:4);
z = pz + [P.cam.posStd*randn(s.rs, 1, 2), deg2rad(P.cam.headingStdDeg)*randn(s.rs)];
z(3) = atan2(sin(z(3)), cos(z(3)));
end
