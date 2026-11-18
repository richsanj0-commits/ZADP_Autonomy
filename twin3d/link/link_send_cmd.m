function cmd = link_send_cmd(L, cmd)
%LINK_SEND_CMD  Publish one command to a robot (topic zadp/bot<id>/cmd, payload as in link/TOPICS.md).
%   cmd = link_send_cmd(L, cmd)
%   cmd fields: id (1..3), truck, action ('goto' | 'turn' | 'dump' | 'stop'), x, y, heading (arena m, rad),
%   speed (m/s); optional dir (turn: +1 CCW, -1 CW, 0 = shortest) and seq. A new command gets the next
%   seq of that robot; a command that already has a seq is sent again unchanged (keep-alive).
%   Returns the command with seq, tFirst (first send) and tSent (this send), seconds on the link clock toc(L.t0).
actions = {'goto', 'turn', 'dump', 'stop'};
assert(any(strcmp(cmd.action, actions)), 'link:action', 'unknown action %s', cmd.action);
assert(cmd.id >= 1 && cmd.id <= L.P.bot.count, 'link:id', 'robot id %d', cmd.id);
if ~isfield(cmd, 'seq') || isempty(cmd.seq) || cmd.seq == 0
    s = L.st('seq'); s(cmd.id) = s(cmd.id) + 1; L.st('seq') = s;
    cmd.seq = s(cmd.id);
    cmd.tFirst = toc(L.t0);                       % first send of this command (keep-alives keep it)
end
if ~isfield(cmd, 'truck') || isempty(cmd.truck), cmd.truck = 0; end
if ~isfield(cmd, 'speed') || isempty(cmd.speed), cmd.speed = L.P.bot.cruiseSpeed; end
r = @(v) round(v, 5);
pl = struct('id', cmd.id, 'truck', cmd.truck, 'seq', cmd.seq, 'action', cmd.action, ...
    'x', r(cmd.x), 'y', r(cmd.y), 'heading', r(cmd.heading), 'speed', cmd.speed);
if strcmp(cmd.action, 'turn') && isfield(cmd, 'dir') && ~isempty(cmd.dir), pl.dir = cmd.dir; end
msg = struct('topic', sprintf(L.P.mqtt.topicCmd, cmd.id), 'payload', pl);
write(L.tcp, uint8([jsonencode(msg) 10]));
cmd.tSent = toc(L.t0);
L.st('nSent') = L.st('nSent') + 1;
last = L.st('last'); last{cmd.id} = cmd; L.st('last') = last;
end
