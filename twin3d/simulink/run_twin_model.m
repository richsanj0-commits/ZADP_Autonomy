function r = run_twin_model(stopTime, opts)
%RUN_TWIN_MODEL  Run zadp_twin.slx for stopTime seconds (default 60) and report what happened.
%   r = run_twin_model(stopTime, opts): r.piles (piles placed at the end), r.coverage (%), r.simTime, r.wall (s),
%   r.renders (3D picture updates by View3D), r.png (last picture).
%   opts.visible ('off': hidden 3D view, the default here), opts.seed (1), opts.planFile ('' = plan_arena),
%   opts.folder (where the model is; default simulink/, built there if missing), opts.png (default: a temp file,
%   so a run never rewrites a file in the repository).
%   Opens no model window; with the view hidden no figure is left open, also when the run fails (onCleanup).
if nargin < 1 || isempty(stopTime), stopTime = 60; end
if nargin < 2, opts = struct(); end
here = fileparts(mfilename('fullpath'));
def = struct('visible', 'off', 'seed', 1, 'planFile', '', 'folder', here, 'png', [tempname '_zadp_twin_last.png']);
f = fieldnames(def);
for i = 1:numel(f), if ~isfield(opts, f{i}), opts.(f{i}) = def.(f{i}); end, end
root = fileparts(here);
addpath(fullfile(root, 'params'), fullfile(root, 'plan'), fullfile(root, 'brain'), fullfile(root, 'bot'), ...
    fullfile(root, 'world3d'), here);
mdl = 'zadp_twin';
file = fullfile(opts.folder, [mdl '.slx']);
if ~isfile(file), build_twin_model(opts.folder); end
if bdIsLoaded(mdl) && ~strcmp(get_param(mdl, 'FileName'), file), close_system(mdl, 0); end
load_system(file);
set_param([mdl '/View3D'], 'PngFile', opts.png, 'Visible', opts.visible, ...
    'PlanFile', opts.planFile);
set_param([mdl '/Brain'], 'Seed', num2str(opts.seed), 'PlanFile', opts.planFile);
cleaner = onCleanup(@() cleanUp(mdl, opts.visible)); %#ok<NASGU>
setappdata(0, 'zadp_twin_renders', 0);
tic0 = tic;
out = sim(mdl, 'StopTime', num2str(stopTime));
wall = toc(tic0);
r = struct('piles', out.piles_placed(end), 'coverage', out.coverage_pct(end), 'simTime', out.sim_time(end), ...
    'wall', wall, 'renders', getappdata(0, 'zadp_twin_renders'), 'png', opts.png);
fprintf('zadp_twin: %g s of sim time, %d piles placed, coverage %.2f %%, %d picture updates, wall %.1f s\n', ...
    r.simTime, r.piles, r.coverage, r.renders, r.wall);
end

function cleanUp(mdl, visible)
% the model is closed without saving (the parameters set above are not written to the file); a hidden view is closed
close_system(mdl, 0);
if strcmp(visible, 'off'), close(findall(0, 'Type', 'figure', 'Tag', 'zadp_twin_view', 'Visible', 'off')); end
end
