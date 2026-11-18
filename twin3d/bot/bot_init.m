function s = bot_init(P, pose0, id)
%BOT_INIT  New robot state. pose0 = [x y heading] (true pose, metres/radians).
%   Fields: id, pose (TRUE), v [vL vR] m/s, wheelAngle [aL aR] rad, enc [L R] counts,
%   odom (encoders only), est (fused), t, slip [kL kR] (hidden per-wheel ground/rotation factors).
if nargin < 3, id = 1; end
pose0 = pose0(:)';
s.id = id;
s.pose = pose0;
s.v = [0 0];
s.wheelAngle = [0 0];
s.enc = [0 0];
s.odom = pose0;
s.est = pose0;
s.t = 0;
s.slip = [1 1] + P.bot.slipStd*randn(1,2);   % start in the stationary slip distribution
end
