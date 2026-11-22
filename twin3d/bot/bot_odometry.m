function pose = bot_odometry(pose, dEnc, P)
%BOT_ODOMETRY  Dead reckoning from encoder count change dEnc = [dL dR] (exact arc, as firmware).
perCount = pi*P.bot.wheelDiameter/P.bot.encoderCPR;
dL = dEnc(1)*perCount; dR = dEnc(2)*perCount;
ds = (dL+dR)/2; dth = (dR-dL)/P.bot.trackEff; th = pose(3);   % effective track (trackScale), as the firmware
if abs(dth) < 1e-9
    pose(1) = pose(1) + ds*cos(th);
    pose(2) = pose(2) + ds*sin(th);
else
    R = ds/dth;
    pose(1) = pose(1) + R*(sin(th+dth) - sin(th));
    pose(2) = pose(2) - R*(cos(th+dth) - cos(th));
end
pose(3) = th + dth;
end
