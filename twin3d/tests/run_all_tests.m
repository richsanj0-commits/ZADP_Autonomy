function ok = run_all_tests()
%RUN_ALL_TESTS  Run every test_*.m in this folder; report pass/fail per test.
%   Each test is a plain script that errors (via assert) on failure.

testDir = fileparts(mfilename('fullpath'));
files   = dir(fullfile(testDir, 'test_*.m'));
failed  = {};

for k = 1:numel(files)
    name = files(k).name(1:end-2);
    try
        runOne(fullfile(testDir, files(k).name));
        fprintf('  PASS  %s\n', name);
    catch err
        fprintf('  FAIL  %s : %s\n', name, err.message);
        failed{end+1} = name; %#ok<AGROW>
    end
end

ok = isempty(failed);
if ok
    disp('ALL TESTS PASSED');
else
    fprintf('FAILED (%d): %s\n', numel(failed), strjoin(failed, ', '));
end
end

function runOne(scriptPath)
% Separate workspace so test variables never leak into the runner.
run(scriptPath);
end
