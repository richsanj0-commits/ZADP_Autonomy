function fig = plot_plan(plan, visible)
% PLOT_PLAN  2D overview of an exported plan: arena, boundary, safe zone, zones, branches, spine, entry, first truck's spots.
%   fig = plot_plan(plan, 'off') draws into a hidden figure (for saving to a file).
if nargin < 2, visible = 'on'; end
fig = figure('Name', ['Plan: ' plan.site], 'Color', 'w', 'Visible', visible); hold on; axis equal;
rectangle('Position', [0 0 plan.arena.w plan.arena.h], 'EdgeColor', [.5 .5 .5], 'LineStyle', ':');
nz = numel(plan.zones); cols = hsv2rgb([(0:nz-1)'/max(nz,1), 0.25*ones(nz,1), ones(nz,1)]);
for i = 1:nz
    z = plan.zones(i); fill(z.poly(:,1), z.poly(:,2), cols(i,:), 'EdgeColor', [.6 .6 .6]);
    c = mean(z.poly, 1); text(c(1), c(2), sprintf('Z%d', z.n), 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    for k = 1:numel(z.branch), b = z.branch{k}; plot(b([1:end 1],1), b([1:end 1],2), '-', 'Color', [.85 .5 0], 'LineWidth', 1.5); end
end
plot(plan.boundary([1:end 1],1), plan.boundary([1:end 1],2), 'k-', 'LineWidth', 1.5);
for k = 1:numel(plan.safe), s = plan.safe{k}; plot(s([1:end 1],1), s([1:end 1],2), '--', 'Color', [0 .5 0], 'LineWidth', 1.2); end
sp = plan.spine.poly; plot(sp([1:end 1],1), sp([1:end 1],2), '-', 'Color', [.3 .3 .3]);
a = plan.spine.inLine; b = plan.spine.outLine;
plot(a(:,1), a(:,2), 'b-', 'LineWidth', 2); text(a(2,1), a(2,2), ' IN', 'Color', 'b');
plot(b(:,1), b(:,2), 'r-', 'LineWidth', 2); text(b(1,1), b(1,2), ' OUT', 'Color', 'r');
plot(plan.entry.pt(1), plan.entry.pt(2), 'ks', 'MarkerFaceColor', 'y', 'MarkerSize', 9);
if ~isempty(plan.trucks) && ~isempty(plan.trucks(1).spots)
    s = plan.trucks(1).spots; plot([s.x], [s.y], '.', 'Color', [.2 .2 .2], 'MarkerSize', 4);
end
title(sprintf('%s (scale %.5g arena m per site m)', plan.site, plan.scale)); xlabel('x [m]'); ylabel('y [m]');
xlim([-0.05 plan.arena.w+0.05]); ylim([-0.05 plan.arena.h+0.05]); hold off;
end
