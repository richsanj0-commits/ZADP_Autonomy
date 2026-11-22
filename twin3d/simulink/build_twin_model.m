function mdl = build_twin_model(saveFolder, opts)
%BUILD_TWIN_MODEL  Create and save zadp_twin.slx programmatically (default: in simulink/).
%   build_twin_model                    -> simulink/zadp_twin.slx (the model users open)
%   build_twin_model(folder)            -> folder/zadp_twin.slx (tests build into a temp folder)
%   opts.planFile ('' = plan/plan_arena.json), opts.visible ('on': the 3D view opens when Run is pressed),
%   opts.stopTime ('600').
%   Blocks: Brain (MATLAB System BrainSystem) -> View3D (MATLAB System View3DSystem, the 3D world), Display blocks for
%   piles placed, trucks dispatched, coverage, robot modes and collisions, a Scope for the coverage, To Workspace
%   blocks (piles_placed, coverage_pct, sim_time) for run_twin_model, and a Stop Simulation block driven by Brain's
%   'done' output (the run ends by itself when every servable spot is filled). Both MATLAB System blocks use
%   "Simulate using: Interpreted execution". Solver: fixed-step, discrete (no states), step P.brain.dt = 0.02 s,
%   stop time 600 s (a full fill needs about 2900 s: set the stop time to inf for that). An annotation on the
%   canvas says how to run it. Model callbacks put twin3d on the path (startup.m next to the simulink folder), so
%   the model also opens and runs in a fresh MATLAB. The model is built in memory without opening a window.
here = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(saveFolder), saveFolder = here; end
if nargin < 2, opts = struct(); end
def = struct('planFile', '', 'visible', 'on', 'stopTime', '600');
f = fieldnames(def);
for i = 1:numel(f), if ~isfield(opts, f{i}), opts.(f{i}) = def.(f{i}); end, end
root = fileparts(here);
addpath(fullfile(root, 'params'), fullfile(root, 'plan'), fullfile(root, 'brain'), fullfile(root, 'bot'), ...
    fullfile(root, 'world3d'), here);
P = twin_params();
mdl = 'zadp_twin';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if isfile(fullfile(saveFolder, [mdl '.slx'])), delete(fullfile(saveFolder, [mdl '.slx'])); end   % else the old file shadows the new name
new_system(mdl);
% path setup in the model itself: a fresh MATLAB (no startup.m run) can open and run it
pathCb = sprintf(['if isempty(which(''BrainSystem'')), run(''%s''); end'], fullfile(root, 'startup.m'));
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', 'FixedStep', num2str(P.brain.dt), ...
    'StopTime', opts.stopTime, 'SaveOutput', 'off', 'SaveTime', 'off', 'PreLoadFcn', pathCb, 'InitFcn', pathCb, ...
    'Description', 'ZADP hardware digital twin: 3 robots (haul trucks at 1:200) fill the dump plan; 3D view of the run.');
sysLib = 'simulink/User-Defined Functions/MATLAB System';
add_block(sysLib, [mdl '/Brain'], 'System', 'BrainSystem', 'SimulateUsing', 'Interpreted execution', ...
    'Position', [60 60 220 360]);
add_block(sysLib, [mdl '/View3D'], 'System', 'View3DSystem', 'SimulateUsing', 'Interpreted execution', ...
    'Position', [420 40 600 380]);
set_param([mdl '/Brain'], 'PlanFile', opts.planFile);
set_param([mdl '/View3D'], 'PlanFile', opts.planFile, 'Visible', opts.visible);
% Brain outputs: 1 pose 2 mode 3 piles 4 coverage 5 time 6 bed 7 truck 8 dump 9 pileTable 10 collisions 11 trucks 12 done
% View3D inputs: 1 pose 2 bed 3 truck 4 dump 5 pileTable 6 piles 7 coverage 8 time 9 collisions 10 trucks
map = [1 1; 6 2; 7 3; 8 4; 9 5; 3 6; 4 7; 5 8; 10 9; 11 10];
for q = 1:size(map, 1)
    add_line(mdl, sprintf('Brain/%d', map(q,1)), sprintf('View3D/%d', map(q,2)), 'autorouting', 'on');
end
% displays, logging, end of run
add_block('simulink/Sinks/Display', [mdl '/Piles placed'], 'Position', [700 40 800 70]);
add_block('simulink/Sinks/Display', [mdl '/Trucks dispatched'], 'Position', [700 90 800 120]);
add_block('simulink/Sinks/Display', [mdl '/Coverage %'], 'Position', [700 140 800 170]);
add_block('simulink/Sinks/Display', [mdl '/Robot modes'], 'Position', [700 190 800 250]);
add_block('simulink/Sinks/Display', [mdl '/Collisions'], 'Position', [700 270 800 300]);
add_block('simulink/Sinks/Scope', [mdl '/Coverage scope'], 'Position', [700 320 740 360]);
set_param([mdl '/Coverage scope'], 'OpenAtSimulationStart', 'off');
add_block('simulink/Sinks/Stop Simulation', [mdl '/Stop when done'], 'Position', [700 390 740 420]);
add_block('simulink/Sinks/To Workspace', [mdl '/log piles'], 'VariableName', 'piles_placed', 'SaveFormat', 'Array', ...
    'Position', [860 40 940 70]);
add_block('simulink/Sinks/To Workspace', [mdl '/log coverage'], 'VariableName', 'coverage_pct', 'SaveFormat', 'Array', ...
    'Position', [860 140 940 170]);
add_block('simulink/Sinks/To Workspace', [mdl '/log time'], 'VariableName', 'sim_time', 'SaveFormat', 'Array', ...
    'Position', [860 340 940 370]);
add_line(mdl, 'Brain/3', 'Piles placed/1', 'autorouting', 'on');
add_line(mdl, 'Brain/11', 'Trucks dispatched/1', 'autorouting', 'on');
add_line(mdl, 'Brain/4', 'Coverage %/1', 'autorouting', 'on');
add_line(mdl, 'Brain/2', 'Robot modes/1', 'autorouting', 'on');
add_line(mdl, 'Brain/10', 'Collisions/1', 'autorouting', 'on');
add_line(mdl, 'Brain/4', 'Coverage scope/1', 'autorouting', 'on');
add_line(mdl, 'Brain/12', 'Stop when done/1', 'autorouting', 'on');
add_line(mdl, 'Brain/3', 'log piles/1', 'autorouting', 'on');
add_line(mdl, 'Brain/4', 'log coverage/1', 'autorouting', 'on');
add_line(mdl, 'Brain/5', 'log time/1', 'autorouting', 'on');
% how to use it, on the canvas
txt = strjoin({ ...
    'ZADP digital twin (twin3d): 3 robots = haul trucks at 1:200 fill the exported dump plan.', ...
    'Press Run. Brain runs the central brain (brain_step: dispatch, section locks, routes, robot physics,', ...
    '  camera fusion, dumping) every 0.02 s and outputs poses, piles, coverage, collisions, trucks, done.', ...
    'View3D draws the 3D world from those signals (Visible = ''on''; closing its window is fine).', ...
    'Displays: piles placed, trucks dispatched, coverage %, robot modes (0 parked, 1 driving, 2 at the dump stop),', ...
    '  collisions (robot-robot + robot-pile + off-site events: must stay 0). Scope: coverage over time.', ...
    'Stop time 600 s (about 25 piles); a full fill takes about 2900 s: set Stop time to inf, the Stop block ends it.', ...
    'Seed and plan: Brain block parameters Seed and PlanFile ('''' = plan/plan_arena.json, or maps/plan_<name>.json).', ...
    'From MATLAB: r = run_twin_model(60) (hidden view, report). Rebuild: build_twin_model. Docs: simulink/README.md.'}, newline);
a = Simulink.Annotation([mdl '/How to run'], 'Text', txt);
a.Position = [60 440 780 600];
a.FontSize = 11; a.HorizontalAlignment = 'left';
save_system(mdl, fullfile(saveFolder, [mdl '.slx']));
close_system(mdl, 0);
end
