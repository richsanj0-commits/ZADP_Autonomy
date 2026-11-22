function cmds = link_from_brain(W, Wprev, L)
%LINK_FROM_BRAIN  Turn route-step changes of the brain into robot commands (the twin leads).
%   cmds = link_from_brain(W, Wprev, L): call it after EVERY brain_step with the world before and after.
%   For each robot whose current route step changed it sends the step to the real robot and returns the
%   commands sent (struct array: id, truck, action, x, y, heading, speed, dir, seq, tFirst, tSent, tSim, step),
%   [] when nothing changed.
%   Mapping of the steps built by brain/route_steps.m:
%     drive -> goto  x, y = end point of the drive, heading = direction of travel
%     turn  -> turn  x, y = where it turns, heading = target heading, dir = planned direction (+1 CCW, -1 CW),
%                    tol = the step's tolerance in deg (P.brain.turnTolDeg, or P.brain.dumpTolDeg at a dump stop)
%     dump  -> dump  x, y, heading = the dump stop (the real robot only waits dumpTime and reports "dumping")
%     hold  -> nothing (no motion)
%   A long drive is cut by route_steps into pieces of <= 0.05 m that the twin passes without braking
%   (step.pass = true, all with the same end point step.p). They are ONE goto: the command is sent when the
%   first piece starts, the later pieces of the same drive send nothing.
cmds = [];
for i = 1:numel(W.bots)
    b = W.bots(i); bp = Wprev.bots(i);
    if ~strcmp(b.mode, 'run'), continue; end
    k = b.k;
    newBlock = ~strcmp(bp.mode, 'run');                  % first step of a new route block
    if ~newBlock
        if k == bp.k, continue; end                      % same step as before
        if b.steps(k-1).pass, continue; end              % next piece of a drive already commanded
    end
    st = b.steps(k);
    c = struct('id', i, 'truck', b.truck, 'action', '', 'x', 0, 'y', 0, 'heading', 0, 'speed', W.P.bot.cruiseSpeed, ...
        'dir', [], 'tol', [], 'seq', 0, 'tFirst', 0, 'tSent', 0, 'tSim', W.t, 'step', k);
    switch st.type
        case 'drive'
            c.action = 'goto'; c.x = st.p(1); c.y = st.p(2); c.heading = st.h;
        case 'turn'
            c.action = 'turn'; c.x = st.p(1); c.y = st.p(2); c.heading = st.h; c.dir = st.dir; c.tol = st.tol;
        case 'dump'
            c.action = 'dump'; c.x = st.p(1); c.y = st.p(2); c.heading = st.h;
        otherwise
            continue
    end
    c = link_send_cmd(L, c);
    if isempty(cmds), cmds = c; else, cmds(end+1) = c; end %#ok<AGROW>
end
end
