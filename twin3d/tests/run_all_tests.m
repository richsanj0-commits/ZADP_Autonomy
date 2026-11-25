function [ok, res] = run_all_tests()
%RUN_ALL_TESTS  Run every test_*.m in this folder; report PASS / SKIP / FAIL per test and the three totals.
%   Each test is a plain script that errors (via assert) on failure. A test that cannot run (Python, numpy,
%   paho-mqtt, node or Simulink missing) prints a line with SKIP and returns without an error.
%   The output of each test is captured and printed when the test ends (not live), then its verdict line:
%     FAIL  the script raised an error
%     SKIP  the script printed "SKIP  <test name>": the whole test did not run. It is NOT counted as passed.
%     PASS  the script returned; "(partial skip: n)" is added when n of its output lines contain SKIP
%   ok  = true when no test failed (a skip does not make ok false, but it is listed and counted on its own).
%   res = struct array, one per test: name, status ('PASS' 'SKIP' 'FAIL'), partialSkips, message, seconds.

testDir = fileparts(mfilename('fullpath'));
files   = dir(fullfile(testDir, 'test_*.m'));
res     = struct('name', {}, 'status', {}, 'partialSkips', {}, 'message', {}, 'seconds', {});

for k = 1:numel(files)
    name = files(k).name(1:end-2);
    t0 = tic;
    [txt, err] = runOne(fullfile(testDir, files(k).name));
    r = struct('name', name, 'status', 'PASS', 'partialSkips', 0, 'message', '', 'seconds', toc(t0));
    if ~isempty(txt), fprintf('%s', txt); end
    if ~isempty(txt) && txt(end) ~= newline, fprintf('\n'); end
    skipLines = regexp(txt, '^.*SKIP.*$', 'match', 'lineanchors', 'dotexceptnewline');
    whole = ~isempty(regexp(txt, ['^\s*SKIP\s+' name '\>'], 'once', 'lineanchors'));
    if ~isempty(err)
        % rule: an error is a failure, whatever else the test printed
        r.status = 'FAIL'; r.message = err.message;
        fprintf('  FAIL  %s : %s\n', name, err.message);
    elseif whole
        % rule: a test that skipped itself as a whole did not check anything, so it is never reported as PASS
        r.status = 'SKIP'; r.message = strtrim(skipLines{1});
        fprintf('  SKIP  %s (whole test not run)\n', name);
    elseif ~isempty(skipLines)
        % rule: a pass with skipped parts says so
        r.partialSkips = numel(skipLines);
        r.message = strjoin(strtrim(skipLines), ' | ');
        fprintf('  PASS  %s (partial skip: %d)\n', name, r.partialSkips);
    else
        fprintf('  PASS  %s\n', name);
    end
    res(end+1) = r; %#ok<AGROW>
end

st = {res.status};
failed  = {res(strcmp(st, 'FAIL')).name};
skipped = {res(strcmp(st, 'SKIP')).name};
partial = {res(strcmp(st, 'PASS') & [res.partialSkips] > 0).name};
nPass   = nnz(strcmp(st, 'PASS'));
ok = isempty(failed);

fprintf('TOTAL %d tests: %d passed (%d of them with a partial skip), %d skipped, %d failed\n', ...
    numel(res), nPass, numel(partial), numel(skipped), numel(failed));
if ~isempty(partial), fprintf('PARTIAL SKIP (%d): %s\n', numel(partial), strjoin(partial, ', ')); end
if ~isempty(skipped), fprintf('SKIPPED (%d): %s\n', numel(skipped), strjoin(skipped, ', ')); end
if ~isempty(failed),  fprintf('FAILED (%d): %s\n', numel(failed), strjoin(failed, ', ')); end
% rule: "ALL TESTS PASSED" is printed only when every test ran completely and passed
if ok && isempty(skipped) && isempty(partial)
    disp('ALL TESTS PASSED');
elseif ok
    disp('NO FAILURES, BUT NOT EVERYTHING RAN (see the SKIP lines)');
end
end

function [runnerTxt_, runnerErrOut_] = runOne(runnerScriptPath_)
% Separate workspace so test variables never leak into the runner. The output is captured with evalc; the
% try/catch sits inside the evalc so the output of a failing test is kept too. The names end in "_" so a
% test script (which runs in this workspace) does not overwrite them.
runnerErr_ = [];
runnerTxt_ = evalc('try, run(runnerScriptPath_); catch runnerErr_, end');
runnerErrOut_ = runnerErr_;
end
