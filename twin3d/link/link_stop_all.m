function link_stop_all(L)
%LINK_STOP_ALL  Send a stop command to every robot (action 'stop'; x, y, heading are ignored by the robot).
for i = 1:L.P.bot.count
    link_send_cmd(L, struct('id', i, 'truck', 0, 'action', 'stop', 'x', 0, 'y', 0, 'heading', 0, 'speed', 0));
end
end
