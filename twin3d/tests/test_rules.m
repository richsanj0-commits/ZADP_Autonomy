% TEST_RULES  Brain rules that a normal fill never exercises (brain/README.md), on the reference arena plan.
%   a. Rule 3 invariant during a 60 s run: every section a true body touches is held by that robot, at every
%      step (monitor e); long drives are released piece by piece only behind the robot
%   b. Rule 7, wait cycle through the queue: robot 1 at its dump stop needs the queue tail that robot 3 holds,
%      robot 3 waits for the slot ahead (robot 2), robot 2 waits for the queue head section robot 1 holds ->
%      exactly one deadlock, flagged after deadlockCycleTime, and brain_run ends the run at once
%   c. Rule 7, no progress: a truck waiting forever at its dump stop with nothing assigned, behind a robot that
%      is held (no wait cycle), is flagged after noProgressTime (the detector used to require an assigned spot)
%   d. W.hold (link syncToReal): a held robot stays at the end of its step; released, it goes on and dumps
%   Prints "test_rules PASSED".
here = fileparts(mfilename('fullpath'));
P = twin_params();
plan = load_plan(fullfile(here, '..', 'plan', 'plan_arena.json'));

% ---- a. section ownership while driving
[W, log] = brain_run(plan, P, struct('seed', 4, 'maxTime', 60));
m = log.mon;
assert(m.ownerSteps == 0, 'a: a body reached a section its robot does not hold (%d steps)', m.ownerSteps);
assert(m.rrOverlapSteps == 0 && m.pileHitSteps == 0 && m.boundSteps == 0, 'a: safety monitors');
assert(size(W.piles, 1) >= 1, 'a: no pile in 60 s');
fprintf('  a. 60 s run: %d piles, section-ownership violations 0, robot-robot 0, pile 0, off-site 0\n', size(W.piles, 1));

% ---- b. forced wait cycle through the queue slots
[W, log] = brain_run(plan, P, struct('seed', 1, 'maxTime', 40, 'beforeStep', @(W) forceCycle(W)));
assert(log.mon.deadlocks == 1 && log.mon.deadlock, 'b: wait cycle not flagged (deadlocks %d)', log.mon.deadlocks);
assert(strcmp(log.endReason, 'deadlock: wait cycle'), 'b: end reason "%s"', log.endReason);
assert(log.tEnd > P.brain.deadlockCycleTime && log.tEnd < P.brain.deadlockCycleTime + 1, 'b: run did not end at the deadlock (t = %.2f s)', log.tEnd);
fprintf('  b. forced wait cycle: 1 deadlock, run ended at t = %.2f s (%s)\n', log.tEnd, log.endReason);

% ---- c. no progress with nothing assigned
T = P.brain.noProgressTime;
[~, log] = brain_run(plan, P, struct('seed', 1, 'maxTime', T + 20, 'beforeStep', @(W) forceStall(W)));
assert(log.mon.deadlocks == 1 && strcmp(log.endReason, 'deadlock: no progress'), 'c: stall not flagged (%s)', log.endReason);
assert(abs(log.tEnd - T) < 1, 'c: flagged at t = %.2f s, expected %.0f s', log.tEnd, T);
fprintf('  c. truck waiting at its dump stop with nothing assigned: deadlock after %.1f s (%s)\n', log.tEnd, log.endReason);

% ---- d. hold: robot 1 held for the first 20 s (it stays at the end of its first step), then released
[W, log] = brain_run(plan, P, struct('seed', 1, 'maxTime', 20, 'beforeStep', @(W) setHold(W, true)));
b = W.bots(1);
assert(strcmp(b.mode, 'run') && b.k == 1 && size(W.piles, 1) == 0, 'd: held robot moved on (mode %s, step %d)', b.mode, b.k);
fprintf('  d. robot 1 held 20 s: still at the end of step 1 of %d, 0 piles\n', numel(b.steps));
[W, log] = brain_run(plan, P, struct('seed', 1, 'maxTime', 60, 'beforeStep', @(W) setHold(W, W.t < 20)));
assert(any(W.pileSpot) && log.mon.deadlocks == 0, 'd: released robot did not dump');
fprintf('  d. released at 20 s: %d piles by 60 s, no deadlock\n', size(W.piles, 1));
disp('test_rules PASSED');

% =====================================================================================
function W = forceCycle(W)
% once, at t = 0: robot 1 leaves slot P1 for a fake dump stop (mode atS) whose exit needs Q3S (held by robot 3,
% parked at P3); P1 becomes free, so robot 2 (P2) wants to move up but QW is still owned by robot 1, and robot 3
% waits for slot P2 (robot 2): 1 -> 3 -> 2 -> 1
if W.t > 0, return; end
q3 = strcmp(W.L.secNames, 'Q3S');
W.bots(1).mode = 'atS'; W.bots(1).slot = 0; W.slotRes(1) = 0;
W.bots(1).R = struct('out', struct('need', q3));
W.bots(1).spot = 1;
end

function W = forceStall(W)
% once, at t = 0: robot 2 runs a step that is complete but held (W.hold, as the link does while the real robot
% has not arrived) and waits for nobody; robot 1 sits at a fake dump stop whose exit needs QM, which robot 2
% holds; robot 3 waits for slot P2 (robot 2). Wait chain 1 -> 2 <- 3 has no cycle and nothing moves.
if W.t > 0, return; end
qm = strcmp(W.L.secNames, 'QM');
W.owner(qm) = 2;
W.bots(1).mode = 'atS'; W.bots(1).slot = 0; W.slotRes(1) = 0; W.bots(1).spot = 1;
W.bots(1).R = struct('out', struct('need', qm));
s = W.adv{2}(1); s.type = 'turn'; s.h = W.bots(2).s.pose(3); s.dir = 1; s.pass = false;   % a turn to where it already points
W.bots(2).mode = 'run'; W.bots(2).job = 'adv'; W.bots(2).steps = s; W.bots(2).k = 1;
W.hold(2) = true;
end

function W = setHold(W, on)
W.hold(1) = on;
end
