function r = run_twin_model(stopTime)
%RUN_TWIN_MODEL  Run simulink/zadp_twin.slx for stopTime seconds (default 60) and report what happened.
%   r = run_twin_model(stopTime): r.piles (piles placed at the end), r.coverage (%), r.simTime, r.wall (s),
%   r.renders (3D picture updates by View3D), r.png (last picture, simulink/zadp_twin_last.png).
%   Opens no model window and no figure (the 3D figure is hidden); nothing is left open.
if nargin < 1, stopTime = 60; end
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(fullfile(root, 'params'), fullfile(root, 'plan'), fullfile(root, 'brain'), fullfile(root, 'bot'), ...
    fullfile(root, 'world3d'), here);
mdl = 'zadp_twin';
if ~isfile(fullfile(here, [mdl '.slx'])), build_twin_model(here); end
load_system(fullfile(here, [mdl '.slx']));
png = fullfile(here, 'zadp_twin_last.png');
set_param([mdl '/View3D'], 'PngFile', png);
cleaner = onCleanup(@() close_system(mdl, 0));
tic0 = tic;
out = sim(mdl, 'StopTime', num2str(stopTime));
wall = toc(tic0);
fig = findall(0, 'Type', 'figure', 'Tag', 'zadp_twin_view');
renders = 0; if ~isempty(fig), renders = getappdata(fig(1), 'renders'); end
r = struct('piles', out.piles_placed(end), 'coverage', out.coverage_pct(end), 'simTime', out.sim_time(end), ...
    'wall', wall, 'renders', renders, 'png', png);
close(fig);
fprintf('zadp_twin: %g s of sim time, %d piles placed, coverage %.2f %%, %d picture updates, wall %.1f s\n', ...
    r.simTime, r.piles, r.coverage, r.renders, r.wall);
end
