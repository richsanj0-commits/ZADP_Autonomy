function mdl = build_twin_model(saveFolder)
%BUILD_TWIN_MODEL  Create and save simulink/zadp_twin.slx programmatically.
%   Blocks: Brain (MATLAB System BrainSystem) -> View3D (MATLAB System View3DSystem, the 3D world), Display blocks for
%   piles placed, coverage, robot modes and collisions, a Scope for the coverage, To Workspace blocks
%   (piles_placed, coverage_pct, sim_time) for run_twin_model. Both MATLAB System blocks use "Simulate using:
%   Interpreted execution". Solver: fixed-step, discrete (no states), step P.brain.dt = 0.02 s, stop time 60 s.
%   The model is built in memory without opening a window.
if nargin < 1, saveFolder = fileparts(mfilename('fullpath')); end
root = fileparts(saveFolder);
addpath(fullfile(root, 'params'), fullfile(root, 'plan'), fullfile(root, 'brain'), fullfile(root, 'bot'), ...
    fullfile(root, 'world3d'), saveFolder);
P = twin_params();
mdl = 'zadp_twin';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if isfile(fullfile(saveFolder, [mdl '.slx'])), delete(fullfile(saveFolder, [mdl '.slx'])); end   % else the old file shadows the new name
new_system(mdl);
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', 'FixedStep', num2str(P.brain.dt), ...
    'StopTime', '60', 'SaveOutput', 'off', 'SaveTime', 'off');
sysLib = 'simulink/User-Defined Functions/MATLAB System';
add_block(sysLib, [mdl '/Brain'], 'System', 'BrainSystem', 'SimulateUsing', 'Interpreted execution', ...
    'Position', [60 60 220 300]);
add_block(sysLib, [mdl '/View3D'], 'System', 'View3DSystem', 'SimulateUsing', 'Interpreted execution', ...
    'Position', [420 40 600 360]);
% Brain outputs: 1 pose 2 mode 3 piles 4 coverage 5 time 6 bed 7 truck 8 dump 9 pileTable 10 collisions
% View3D inputs: 1 pose 2 bed 3 truck 4 dump 5 pileTable 6 piles 7 coverage 8 time 9 collisions
map = [1 1; 6 2; 7 3; 8 4; 9 5; 3 6; 4 7; 5 8; 10 9];
for q = 1:size(map, 1)
    add_line(mdl, sprintf('Brain/%d', map(q,1)), sprintf('View3D/%d', map(q,2)), 'autorouting', 'on');
end
% displays and logging
add_block('simulink/Sinks/Display', [mdl '/Piles placed'], 'Position', [700 40 800 70]);
add_block('simulink/Sinks/Display', [mdl '/Coverage %'], 'Position', [700 100 800 130]);
add_block('simulink/Sinks/Display', [mdl '/Robot modes'], 'Position', [700 160 800 220]);
add_block('simulink/Sinks/Display', [mdl '/Collisions'], 'Position', [700 240 800 270]);
add_block('simulink/Sinks/Scope', [mdl '/Coverage scope'], 'Position', [700 300 740 340]);
set_param([mdl '/Coverage scope'], 'OpenAtSimulationStart', 'off');
add_block('simulink/Sinks/To Workspace', [mdl '/log piles'], 'VariableName', 'piles_placed', 'SaveFormat', 'Array', ...
    'Position', [860 40 940 70]);
add_block('simulink/Sinks/To Workspace', [mdl '/log coverage'], 'VariableName', 'coverage_pct', 'SaveFormat', 'Array', ...
    'Position', [860 100 940 130]);
add_block('simulink/Sinks/To Workspace', [mdl '/log time'], 'VariableName', 'sim_time', 'SaveFormat', 'Array', ...
    'Position', [860 340 940 370]);
add_line(mdl, 'Brain/3', 'Piles placed/1', 'autorouting', 'on');
add_line(mdl, 'Brain/4', 'Coverage %/1', 'autorouting', 'on');
add_line(mdl, 'Brain/2', 'Robot modes/1', 'autorouting', 'on');
add_line(mdl, 'Brain/10', 'Collisions/1', 'autorouting', 'on');
add_line(mdl, 'Brain/4', 'Coverage scope/1', 'autorouting', 'on');
add_line(mdl, 'Brain/3', 'log piles/1', 'autorouting', 'on');
add_line(mdl, 'Brain/4', 'log coverage/1', 'autorouting', 'on');
add_line(mdl, 'Brain/5', 'log time/1', 'autorouting', 'on');
save_system(mdl, fullfile(saveFolder, [mdl '.slx']));
close_system(mdl, 0);
end
