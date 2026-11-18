% STARTUP  Put the twin3d subfolders on the MATLAB path.
%   Run once per MATLAB session:  run('E:\catrover_dumpautonomy\twin3d\startup.m')
%   Safe to run repeatedly (addpath ignores folders already on the path).

twinRoot = fileparts(mfilename('fullpath'));          % folder holding this file
subDirs  = {'params', 'plan', 'brain', 'bot', 'world3d', 'simulink', 'link', 'tests'};

added = {};
for k = 1:numel(subDirs)
    d = fullfile(twinRoot, subDirs{k});
    if isfolder(d)
        addpath(d);
        added{end+1} = subDirs{k}; %#ok<AGROW>
    end
end

fprintf('twin3d digital twin | MATLAB %s\n', version);
fprintf('root: %s\n', twinRoot);
fprintf('on path: %s\n', strjoin(added, ', '));

clear twinRoot subDirs added d k
