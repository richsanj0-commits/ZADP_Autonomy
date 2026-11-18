function demo_3d(maxSimTime)
%DEMO_3D  Run the brain for the first 240 s of sim time (seed 1), then write
%   world3d/demo_3d.png  an orbit view in the middle of the run with several piles and a truck tipping, and
%   world3d/demo_3d.mp4  about 30 s of video of the same 240 s, orbit view slowly turning (x8 real time).
%   demo_3d(maxSimTime) changes the length of the run (default 240).
if nargin < 1, maxSimTime = 240; end
here = fileparts(mfilename('fullpath'));
P = twin_params();
plan = load_plan(fullfile(here, '..', 'plan', 'plan_arena.json'));
fprintf('demo_3d: running the brain for %g s ...\n', maxSimTime);
[~, log] = brain_run(plan, P, struct('seed', 1, 'maxTime', maxSimTime));
fprintf('  %d piles, %d trucks, wall %.1f s\n', size(log.piles, 1), log.trucks, log.wall);

% ---- PNG: the last dump that ends before the end of the run, caught with the bed fully up and the pile full-size
pt = [log.pileRec.t]; k = find(pt <= maxSimTime - 2, 1, 'last');
tShot = pt(k) - 0.25*P.truck.dumpTime;           % bed fully up, pile full size, about to lower
st = world_play(log, plan, P, struct('speed', 1, 'tStart', tShot, 'maxFrames', 1, 'visible', 'off'));
h = st.world;
% camera: 3/4 view of the tipping truck from the south side (65 degrees from its heading), looking at the middle
% between the truck and its new pile, close enough to show the truck and the piles beside it
bot = log.pileRec(k).bot; i0 = find(log.t <= tShot, 1, 'last'); pz = log.pose(i0, :, bot);
sgn = 1; if sin(pz(3) + deg2rad(65)) > 0, sgn = -1; end
ac = pz(3) + sgn*deg2rad(65); el = deg2rad(33); d = 1.05;
tg = [pz(1:2) - 0.04*[cos(pz(3)) sin(pz(3))], 0.01];
tg(1:2) = tg(1:2) - 0.10*[-sin(ac) cos(ac)];             % scene a little to the right of the HUD
set(h.ax, 'CameraViewAngle', 24, 'CameraTarget', tg, 'CameraPosition', tg + d*[cos(ac)*cos(el), sin(ac)*cos(el), sin(el)]);
camlight(h.headlight, 'headlight');
drawnow;
fr = getframe(st.fig);
imwrite(fr.cdata, fullfile(here, 'demo_3d.png'));
close(st.fig);
fprintf('  wrote demo_3d.png (%dx%d), shot at t = %.1f s\n', size(fr.cdata, 2), size(fr.cdata, 1), tShot);

% ---- MP4: whole run, x8 so 240 s become 30 s
o = struct('speed', maxSimTime/30, 'fps', 30, 'tStart', 0, 'tEnd', log.tEnd, 'view', 'orbit', 'orbitRate', 1.5, 'quality', 35, ...
    'file', fullfile(here, 'demo_3d.mp4'));
s = world_play(log, plan, P, o);
fprintf('  wrote %s: %d frames, %.1f s of video, %.1f kB, %.1f frames/s while rendering\n', s.file, s.frames, ...
    s.videoSeconds, s.bytes/1e3, s.fps);
end
