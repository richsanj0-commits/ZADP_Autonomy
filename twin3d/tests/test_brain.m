% TEST_BRAIN  Phase 4 central brain: 3 robots fill the reference arena plan (plan/plan_arena.json), seeds 1, 2, 3.
%   Per seed (check_brain_run): zero robot-robot overlaps, zero pile overlaps, zero boundary violations, zero
%   section-ownership violations (Rule 3), no deadlock, no failed exit re-plan, run ended normally (Rule 8), every
%   pile within 15 mm of its spot, piles >= 98 % of all spots (123 of 125), zone lock never violated, lane spots last and alone
%   (Rule 4), and at some moment two robots inside the site at once.
%   Other maps: test_maps. Prints "test_brain PASSED". Takes a few minutes (one full fill per seed).
here = fileparts(mfilename('fullpath'));
P = twin_params();
plan = load_plan(fullfile(here, '..', 'plan', 'plan_arena.json'));
tAll = tic;
for seed = 1:3
    check_brain_run(plan, P, seed, 0.98, 'arena');
end
fprintf('  test_brain runtime %.0f s\n', toc(tAll));
disp('test_brain PASSED');
