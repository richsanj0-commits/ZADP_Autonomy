function s = pose_fusion(s, z, P)
%POSE_FUSION  Between fixes s.est follows the encoder increments (done in bot_step);
%   a camera fix z (non-empty) pulls s.est toward it with gain P.cam.fusionGain (heading wrapped).
if isempty(z), return; end
e = z(:)' - s.est;
e(3) = atan2(sin(e(3)), cos(e(3)));
s.est = s.est + P.cam.fusionGain*e;
s.est(3) = atan2(sin(s.est(3)), cos(s.est(3)));
end
