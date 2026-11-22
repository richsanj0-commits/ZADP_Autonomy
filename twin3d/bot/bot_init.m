function s = bot_init(P, pose0, id, rs)
%BOT_INIT  New robot state. pose0 = [x y heading] (true pose, metres/radians).
%   Fields: id, pose (TRUE), v [vL vR] m/s, wheelAngle [aL aR] rad, enc [L R] counts,
%   odom (encoders only), est (fused), t, slip [kL kR] (hidden per-wheel ground/rotation factors),
%   hist (short history [t, true pose, odom] for the camera latency, see cam_measure / pose_fusion),
%   rs (random stream for slip and camera noise; default the global stream; brain_init passes the
%   world's own stream so a run depends on its seed only).
if nargin < 3, id = 1; end
if nargin < 4, rs = RandStream.getGlobalStream; end
pose0 = pose0(:)';
s.id = id;
s.pose = pose0;
s.v = [0 0];
s.wheelAngle = [0 0];
s.enc = [0 0];
s.odom = pose0;
s.est = pose0;
s.t = 0;
s.rs = rs;
s.slip = [1 1] + P.bot.slipStd*randn(rs, 1, 2);   % start in the stationary slip distribution
s.hist = [0, pose0, pose0];
end
