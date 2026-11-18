function fig = brain_plot(W, log, plan, file)
%BRAIN_PLOT  One figure of a brain run: the plan, the apron loop and sections, piles as filled circles
%   (unfilled spots marked x), the three robots' true paths in different colours. Saves a PNG when
%   file is given (hidden figure, white background).
if nargin < 4, file = ''; end
vis = 'on'; if ~isempty(file), vis = 'off'; end
fig = plot_plan(plan, vis);
set(fig, 'Position', [100 100 900 1100]);
ax = gca; hold(ax, 'on');
L = W.L;
% apron, queue slots and loop
A = L.apronRect;
rectangle(ax, 'Position', [A(1) A(3) A(2)-A(1) A(4)-A(3)], 'EdgeColor', [.4 .4 .4], 'LineStyle', '-');
loop = [L.xIn L.yQ; L.xIn 0; NaN NaN; L.xOut 0; L.xOut L.yR; L.xE L.yR; L.xE L.yQ; L.xIn L.yQ];
plot(ax, loop(:,1), loop(:,2), ':', 'Color', [.3 .3 .3], 'LineWidth', 1);
plot(ax, L.slot(:,1), L.slot(:,2), 'ks', 'MarkerSize', 7);
text(ax, L.slot(:,1), L.slot(:,2) - 0.03, {'P1','P2','P3'}, 'HorizontalAlignment', 'center', 'FontSize', 8);
text(ax, 0.02, -0.2, 'apron / parking', 'FontSize', 9, 'Color', [.3 .3 .3]);
% junction box
plot(ax, [L.xSpL-L.jSide L.xSpR+L.jSide L.xSpR+L.jSide L.xSpL-L.jSide L.xSpL-L.jSide], ...
    [L.yJ1 L.yJ1 L.yJ2 L.yJ2 L.yJ1], '--', 'Color', [.6 .2 .6]);
text(ax, L.xSpR + L.jSide, L.yJ2 + 0.01, 'J', 'Color', [.6 .2 .6]);
% robot paths
cols = [0.85 0.33 0.10; 0.00 0.45 0.74; 0.47 0.67 0.19];
h = gobjects(1, size(log.pose, 3));
for i = 1:size(log.pose, 3)
    h(i) = plot(ax, log.pose(:,1,i), log.pose(:,2,i), '-', 'Color', [cols(i,:) 0.55], 'LineWidth', 0.8, ...
        'DisplayName', sprintf('robot %d (%d trips)', i, log.trips(i)));
end
% piles and unfilled spots
th = linspace(0, 2*pi, 40);
for k = 1:size(W.piles, 1)
    r = W.pileR(k);          % actual radius of this pile
    fill(ax, W.piles(k,1) + r*cos(th), W.piles(k,2) + r*sin(th), W.P.pile.color, ...
        'EdgeColor', [.35 .25 .15], 'FaceAlpha', 0.85);
end
st = {W.spots.status};
u = find(~strcmp(st, 'placed'));
if ~isempty(u)
    hu = plot(ax, [W.spots(u).x], [W.spots(u).y], 'rx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'unfilled spot');
    h = [h hu];
end
m = log.mon;
title(ax, sprintf(['Brain run: %d / %d piles, %.0f s sim | overlaps %d, pile hits %d, off-site %d, ' ...
    'deadlocks %d | min robot gap %.1f mm'], size(W.piles,1), numel(W.spots), log.tEnd, ...
    m.rrOverlapEvents, m.pileHitEvents, m.boundEvents, m.deadlocks, 1000*m.minRR), 'FontSize', 9);
legend(ax, h, 'Location', 'northoutside', 'NumColumns', 4);
xlim(ax, [-0.03 1.03]); ylim(ax, [-P_apron(W)-0.03 1.03]);
try, theme(fig, 'light'); catch, end
set(fig, 'Color', 'w'); set(ax, 'Color', 'w');
hold(ax, 'off');
if ~isempty(file)
    exportgraphics(fig, file, 'Resolution', 150, 'BackgroundColor', 'white');
    close(fig);
end
end

function a = P_apron(W)
a = W.L.apron;
end
