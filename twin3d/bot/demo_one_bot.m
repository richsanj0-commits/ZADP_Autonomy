function res = demo_one_bot(outFile, doPlot)
%DEMO_ONE_BOT  One robot: entry -> IN lane to the depth of zone 1's deepest spot -> that spot -> OUT lane -> exit.
%   res = demo_one_bot()               plots in a visible figure
%   res = demo_one_bot('file.png')     saves a PNG from a hidden figure (white background)
%   res = demo_one_bot('', false)      no plotting (used by tests)
if nargin < 1, outFile = ''; end
if nargin < 2, doPlot = true; end
P = twin_params();
root = fileparts(fileparts(mfilename('fullpath')));
plan = load_plan(fullfile(root, 'plan', 'plan_arena.json'));

% deepest spot of the first zone = largest distance along the entry normal
sp = plan.trucks(1).spots; sp = sp([sp.zone] == plan.zones(1).n);
[~, i] = max([sp.x]*plan.entry.n(1) + [sp.y]*plan.entry.n(2));
spot = [sp(i).x sp(i).y];
depth = spot(1:2)*plan.entry.n(:);
inL = plan.spine.inLine; outL = plan.spine.outLine;
yDeep = spot(2);
wp = [inL(1,1), yDeep;           % IN lane at the spot's depth
      spot;                      % the spot
      outL(1,1), yDeep;          % OUT lane at the same depth
      outL(end,:)];              % exit
names = {'IN lane', 'spot', 'OUT lane', 'exit'};

pose0 = [plan.entry.pt, atan2(plan.entry.n(2), plan.entry.n(1))];
s = bot_init(P, pose0, 1);
opts = struct('useCamera', true, 'timeout', 60);
trueP = s.pose; odomP = s.odom; estP = s.est; arrivedAll = true; tLeg = zeros(1,size(wp,1));
for k = 1:size(wp,1)
    [s, lg] = bot_goto(s, wp(k,:), P, opts);
    arrivedAll = arrivedAll && lg.arrived;
    trueP = [trueP; lg.truePose]; odomP = [odomP; lg.odom]; estP = [estP; lg.est]; %#ok<AGROW>
    tLeg(k) = lg.tArrive;
end
res.arrived = arrivedAll; res.tTrip = s.t; res.finalError = hypot(s.pose(1)-wp(end,1), s.pose(2)-wp(end,2));
res.truePath = trueP(:,1:2); res.odomPath = odomP(:,1:2); res.estPath = estP(:,1:2);
res.waypoints = wp; res.spot = spot; res.depth = depth; res.tLeg = tLeg;
res.inArena = all(trueP(:,1) >= 0 & trueP(:,1) <= P.arena.w & trueP(:,2) >= 0 & trueP(:,2) <= P.arena.h);
fprintf('demo_one_bot: arrived=%d, trip time %.1f s, final true-position error at exit %.1f mm (odometry-only says %.1f mm)\n', ...
    res.arrived, res.tTrip, 1000*res.finalError, 1000*hypot(s.odom(1)-wp(end,1), s.odom(2)-wp(end,2)));

if doPlot
    vis = 'on'; if ~isempty(outFile), vis = 'off'; end
    fig = plot_plan(plan, vis); hold on;
    plot(res.truePath(:,1), res.truePath(:,2), 'k-', 'LineWidth', 1.6, 'DisplayName', 'true');
    plot(res.odomPath(:,1), res.odomPath(:,2), 'm--', 'LineWidth', 1.2, 'DisplayName', 'odometry only');
    plot(res.estPath(:,1), res.estPath(:,2), 'c-', 'LineWidth', 1.0, 'DisplayName', 'fused (camera+encoders)');
    plot(wp(:,1), wp(:,2), 'g^', 'MarkerFaceColor', 'g', 'DisplayName', 'waypoints');
    legend('show', 'Location', 'southwest');
    title(sprintf('One robot: true / odometry / fused path (final error %.1f mm)', 1000*res.finalError));
    try, theme(fig, 'light'); catch, end
    set(fig, 'Color', 'w'); set(findall(fig, 'Type', 'axes'), 'Color', 'w');
    if ~isempty(outFile)
        exportgraphics(fig, outFile, 'Resolution', 150, 'BackgroundColor', 'white');
        close(fig);
    end
end
end
