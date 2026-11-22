% TEST_RUN_REAL  run_real('dry'): the one-command real-robot launcher on its full code path without hardware
%   (loopback bridge, fake_bot.py robots without their own camera, aruco_node.py --sim feeding synthetic marker
%   pixels through the real camera geometry). Prints "test_run_real PASSED".
%   a. a 20 s dry run: preflight, corners, ready and park checks pass; twin monitors all 0; every robot within
%      30 mm of its twin; commands acknowledged; no hold timeout; the live view was driven; helpers gone (PID)
%   b. robot 2 put 60 mm off its slot: error run_real:park naming robot 2; helpers gone
%   c. corner 11 hidden from the camera: error run_real:corners naming id 11; helpers gone
%   d. the view closed during the run (as a user would): error link:aborted, helpers gone, no figure left
%   If Python or numpy is missing the test prints SKIP and passes.
P = twin_params();
[exe, ~, pyVer] = link_python();
if isempty(exe) || system(sprintf('"%s" -c "import numpy"', exe)) ~= 0
    fprintf('  SKIP  test_run_real: Python with numpy is not available\n');
    return
end
fprintf('  Python %s\n', pyVer);

pidsBefore = listPython();

% ---- a. full dry run
R = run_real('dry', struct('maxTime', 20));
m = R.mon; lk = R.lk;
assert(R.checks.preflight && R.checks.corners && R.checks.ready, 'a: checks');
assert(m.rrOverlapSteps == 0 && m.pileHitSteps == 0 && m.boundSteps == 0 && m.deadlocks == 0 && m.ownerSteps == 0, ...
    'a: twin safety monitors (rr %d, pile %d, bound %d, deadlock %d, owner %d)', m.rrOverlapSteps, m.pileHitSteps, ...
    m.boundSteps, m.deadlocks, m.ownerSteps);
assert(all(lk.worstErrBot < 0.030), 'a: twin-vs-real error %s mm', mat2str(round(1000*lk.worstErrBot, 1)));
assert(lk.nCmd >= 5 && numel(lk.latencyMs) >= lk.nCmd - 1, 'a: %d commands, %d acknowledged', lk.nCmd, numel(lk.latencyMs));
assert(lk.holdTimeouts == 0, 'a: %d hold timeouts', lk.holdTimeouts);
assert(lk.nCam > 10*20, 'a: %d camera poses', lk.nCam);
assert(R.nView > 10, 'a: live view refreshed only %d times', R.nView);
assert(contains(R.camLog, 'homography ready'), 'a: camera log');
assert(isempty(setdiff(listPython(), pidsBefore)), 'a: helper processes still alive after run_real returned');
fprintf(['  a. dry run 20 s: park worst %.1f mm, %d commands, error mean %.1f mm worst %.1f mm, latency mean %.0f ms, ' ...
    '%d camera poses, %d view frames, monitors 0, helpers gone\n'], 1000*max([R.checks.park.dist]), lk.nCmd, ...
    1000*lk.meanErr, 1000*lk.worstErr, lk.meanLatencyMs, lk.nCam, R.nView);

% ---- b. a robot off its parking slot
init = brain_layout(load_plan(fullfile(fileparts(mfilename('fullpath')), '..', 'plan', 'plan_arena.json')), P).slot;
init(2, 1) = init(2, 1) + 0.06;
try
    run_real('dry', struct('maxTime', 5, 'init', init));
    error('test:noError', 'b: run_real accepted a robot 60 mm off its slot');
catch err
    assert(strcmp(err.identifier, 'run_real:park') && contains(err.message, 'robot 2') && ~contains(err.message, 'robot 1 ') ...
        && ~contains(err.message, 'robot 3'), 'b: wrong error: %s (%s)', err.message, err.identifier);
    msg = strsplit(err.message, newline);
    fprintf('  b. robot 2 off its slot -> %s\n     %s\n', err.identifier, strtrim(msg{2}));
end
assert(isempty(setdiff(listPython(), pidsBefore)), 'b: helper processes left');

% ---- c. a corner marker hidden
try
    run_real('dry', struct('maxTime', 5, 'simHideCorner', '11'));
    error('test:noError', 'c: run_real ran with a hidden corner');
catch err
    assert(strcmp(err.identifier, 'run_real:corners') && contains(err.message, 'missing ids [11]'), 'c: wrong error: %s', err.message);
    fprintf('  c. corner 11 hidden -> %s ("missing ids [11]" in the camera log)\n', err.identifier);
end
assert(isempty(setdiff(listPython(), pidsBefore)), 'c: helper processes left');

% ---- d. closing the view stops the run (close() runs the figure's CloseRequestFcn, as the window's X button does)
try
    run_real('dry', struct('maxTime', 30, 'viewHook', @(W, fig) closeAt(W, fig, 8)));
    error('test:noError', 'd: run_real did not stop when the view was closed');
catch err
    assert(strcmp(err.identifier, 'link:aborted'), 'd: wrong error: %s (%s)', err.message, err.identifier);
    fprintf('  d. view closed during the run -> %s: %s\n', err.identifier, err.message);
end
assert(isempty(setdiff(listPython(), pidsBefore)), 'd: helper processes left');
assert(isempty(findall(0, 'type', 'figure', '-regexp', 'Name', '^ZADP twin with the real robots')), 'd: figure left');
fprintf('test_run_real PASSED\n');

function closeAt(W, fig, t)
if W.t >= t && isgraphics(fig), close(fig); end
end

function pids = listPython()
% PIDs of running python.exe processes (to prove nothing was left behind)
[~, out] = system('tasklist /NH /FO CSV');
tok = regexp(out, '"python[^"]*\.exe","(\d+)"', 'tokens');
pids = cellfun(@(c) str2double(c{1}), tok);
end
