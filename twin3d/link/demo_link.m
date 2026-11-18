function varargout = demo_link(opts)
%DEMO_LINK  The twin leads, fake robots follow: about 120 s of sim time (= wall time) with the link attached.
%   Starts bridge.py and fake_bot.py in the background (killed at the end, also on error), runs
%   brain_run_linked, prints commands sent, telemetry received, twin-vs-real position error, command round-trip
%   latency, and saves link\demo_link.png (twin paths solid, "real" = camera paths dashed, error vs time).
%   Takes as long as the sim time it covers (about 2 min); no hardware, no broker needed (loopback mode).
%   demo_link(opts): opts.maxTime (120), opts.syncToReal (false), opts.mode ('auto' | 'loopback'), opts.seed (1).
%   [W, log, lk, procs] = demo_link(...) also returns the results.
if nargin < 1, opts = struct(); end
def = struct('maxTime', 120, 'syncToReal', false, 'mode', 'auto', 'seed', 1);
f = fieldnames(def);
for i = 1:numel(f), if ~isfield(opts, f{i}), opts.(f{i}) = def.(f{i}); end, end
here = fileparts(mfilename('fullpath'));
P = twin_params();
plan = load_plan(fullfile(fileparts(here), 'plan', 'plan_arena.json'));

[exe, hasPaho, ver] = link_python();
if isempty(exe), disp('demo_link: SKIP, Python is not available'); return; end
fprintf('demo_link: Python %s, paho-mqtt %s\n', ver, yesno(hasPaho));

procs = link_launch(P, struct('mode', opts.mode));
killer = onCleanup(@() link_kill(procs)); %#ok<NASGU>
fprintf('  bridge mode: %s (port %d), PIDs %s\n', procs.mode, procs.port, mat2str(procs.pids));
L = link_open(P, struct('port', procs.port));
guard = onCleanup(@() link_close(L)); %#ok<NASGU>

[W, log, lk] = brain_run_linked(plan, P, struct('link', L, 'maxTime', opts.maxTime, 'seed', opts.seed, ...
    'syncToReal', opts.syncToReal));
m = log.mon;
fprintf('demo_link: %.0f s sim, syncToReal %s, bridge mode %s\n', log.tEnd, yesno(opts.syncToReal), lk.mode);
fprintf('  commands sent %d, telemetry received %d, camera poses received %d\n', lk.nCmd, lk.nTel, lk.nCam);
fprintf('  twin vs real position error: mean %.1f mm, worst %.1f mm (per robot mean %s mm, worst %s mm)\n', ...
    1000*lk.meanErr, 1000*lk.worstErr, mat2str(round(1000*lk.meanErrBot, 1)), mat2str(round(1000*lk.worstErrBot, 1)));
fprintf('  twin vs real heading error: worst %.1f deg\n', rad2deg(max(lk.herr(:))));
fprintf('  command round trip: mean %.1f ms, worst %.1f ms (%d of %d acknowledged)\n', lk.meanLatencyMs, lk.maxLatencyMs, ...
    numel(lk.latencyMs), lk.nCmd);
fprintf('  twin safety: robot-robot overlaps %d, pile hits %d, boundary %d, deadlocks %d; brain max lag behind the wall clock %.0f ms\n', ...
    m.rrOverlapSteps, m.pileHitSteps, m.boundSteps, m.deadlocks, 1000*lk.maxLag);
if opts.syncToReal
    fprintf('  twin held for the real robot: %s s per robot, hold timeouts %d\n', mat2str(round(lk.holdTime, 1)), lk.holdTimeouts);
end

plotLink(plan, lk, fullfile(here, 'demo_link.png'));
disp('  saved link/demo_link.png');
if nargout, varargout = {W, log, lk, procs}; end
end

function plotLink(plan, lk, file)
fig = figure('Visible', 'off', 'Position', [100 100 1100 520]);
try, theme(fig, 'light'); catch, end %#ok<NOCOM>
col = [0.85 0.33 0.10; 0.00 0.45 0.74; 0.47 0.67 0.19];
ax = subplot(1, 2, 1); hold(ax, 'on'); axis(ax, 'equal');
bd = plan.boundary; plot(ax, bd([1:end 1], 1), bd([1:end 1], 2), '-', 'Color', [0.6 0.6 0.6]);
rectangle(ax, 'Position', [0 -0.25 1 0.25], 'EdgeColor', [0.8 0.8 0.8]);
nb = size(lk.twin, 3); h = gobjects(1, 2*nb);
for i = 1:nb
    h(i) = plot(ax, lk.twin(:,1,i), lk.twin(:,2,i), '-', 'Color', col(i,:), 'LineWidth', 2);
    h(nb+i) = plot(ax, lk.real(:,1,i), lk.real(:,2,i), '--', 'Color', col(i,:)*0.6, 'LineWidth', 1.2);
end
xlim(ax, [-0.05 1.05]); ylim(ax, [-0.3 1.05]); xlabel(ax, 'x (m)'); ylabel(ax, 'y (m)');
title(ax, 'paths: twin solid, real (camera) dashed');
legend(ax, h([1:nb nb+1]), [arrayfun(@(i) sprintf('twin %d', i), 1:nb, 'UniformOutput', false), {'real (dashed)'}], 'Location', 'southoutside', 'Orientation', 'horizontal');
ax2 = subplot(1, 2, 2); hold(ax2, 'on');
for i = 1:nb, plot(ax2, lk.t, 1000*lk.err(:,i), '-', 'Color', col(i,:)); end
yline(ax2, 30, ':', '30 mm');
xlabel(ax2, 'sim time (s)'); ylabel(ax2, 'twin vs real position error (mm)'); grid(ax2, 'on');
title(ax2, sprintf('mean %.1f mm, worst %.1f mm', 1000*lk.meanErr, 1000*lk.worstErr));
legend(ax2, arrayfun(@(i) sprintf('robot %d', i), 1:nb, 'UniformOutput', false), 'Location', 'northwest');
exportgraphics(fig, file, 'Resolution', 130);
close(fig);
end

function s = yesno(b)
if b, s = 'yes'; else, s = 'no'; end
end
