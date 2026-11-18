function [X, Y] = body_outline(poses, P, infl, step)
%BODY_OUTLINE  Perimeter points of the robot body (axle pose [x y th], K-by-3), inflated by infl.
%   Points every <= step metres along the rectangle edges. X, Y are K-by-M.
if nargin < 3, infl = 0; end
if nargin < 4, step = 0.01; end
persistent keys LX LY
k = [infl, step, P.bot.axleToNose, P.bot.axleToBack, P.bot.chassisWidth];
j = 0;
for q = 1:numel(keys), if isequal(keys{q}, k), j = q; break; end, end
if j == 0
    a = P.bot.axleToNose + infl; b = P.bot.axleToBack + infl; w = P.bot.chassisWidth/2 + infl;
    nL = max(2, ceil((a+b)/step)+1); nW = max(2, ceil(2*w/step)+1);
    sL = linspace(-b, a, nL); sW = linspace(-w, w, nW);
    keys{end+1} = k;
    LX{end+1} = [sL, a*ones(1,nW-2), fliplr(sL), -b*ones(1,nW-2)];
    LY{end+1} = [-w*ones(1,nL), sW(2:end-1), w*ones(1,nL), fliplr(sW(2:end-1))];
    j = numel(keys);
end
lx = LX{j}; ly = LY{j};
c = cos(poses(:,3)); s = sin(poses(:,3));
X = poses(:,1) + c.*lx - s.*ly;
Y = poses(:,2) + s.*lx + c.*ly;
end
