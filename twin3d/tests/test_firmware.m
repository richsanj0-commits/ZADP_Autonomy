% TEST_FIRMWARE  The firmware and the camera node agree with the twin. Prints "test_firmware PASSED".
%   a. link/py/check_numbers.py: every number in firmware/zadp_bot/include/config.h equals twin_params
%      (via a fresh link_config.json), including TRACK_SCALE = P.bot.trackScale and the parking start poses
%   b. firmware/zadp_bot/test/host/js_mirror/mirror_test.js (node): motion_core.h controller, command semantics,
%      watchdog, latency-compensated fusion against a simulated robot (SKIP if node is missing)
%   c. link/py/test_aruco_geometry.py: the camera node's geometry on synthetic marker pixels (numpy only)
P = twin_params();
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
py = fullfile(fileparts(here), 'link', 'py');
exe = link_python();
if isempty(exe)
    fprintf('  SKIP  test_firmware: Python is not available\n');
    return
end
cfg = fullfile(tempdir, sprintf('zadp_cfg_test_%d.json', randi(1e6)));
link_write_config(P, cfg);
cleaner = onCleanup(@() delete(cfg));

[st, out] = system(sprintf('"%s" "%s" --config "%s"', exe, fullfile(py, 'check_numbers.py'), cfg));
assert(st == 0, 'a: config.h differs from twin_params:\n%s', out);
n = numel(regexp(out, '\sok\s*$', 'lineanchors'));
fprintf('  a. config.h vs twin_params: %d numbers, all match\n', n);

[stn, ~] = system('node --version');
if stn ~= 0
    fprintf('  b. SKIP: node not found, mirror_test.js not run\n');
else
    [st, out] = system(sprintf('node "%s"', fullfile(root, 'firmware', 'zadp_bot', 'test', 'host', 'js_mirror', 'mirror_test.js')));
    tok = regexp(out, '(\d+) passed, (\d+) failed', 'tokens', 'once');
    assert(st == 0 && ~isempty(tok) && strcmp(tok{2}, '0'), 'b: mirror_test.js failed:\n%s', out);
    fprintf('  b. motion_core JS mirror: %s passed, 0 failed\n', tok{1});
end

[sn, ~] = system(sprintf('"%s" -c "import numpy"', exe));
if sn ~= 0
    fprintf('  c. SKIP: numpy not installed\n');
else
    [st, out] = system(sprintf('"%s" "%s" --config "%s"', exe, fullfile(py, 'test_aruco_geometry.py'), cfg));
    tok = regexp(out, '(\d+) passed, (\d+) failed', 'tokens', 'once');
    assert(st == 0 && ~isempty(tok) && strcmp(tok{2}, '0'), 'c: test_aruco_geometry.py failed:\n%s', out);
    l = regexp(out, 'same plane[^\n]*', 'match', 'once');
    fprintf('  c. camera geometry: %s passed, 0 failed (%s)\n', tok{1}, strtrim(l));
end
fprintf('test_firmware PASSED\n');
