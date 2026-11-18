function [st, ok, why] = route_steps(st, op, arg, piles, L, P, dirs, tol)
%ROUTE_STEPS  Append one move to a route under construction and check its swept body.
%   st.pose = current pose [x y th]; st.steps = struct array of steps.
%   op 'drive': arg = target point [x y]; heading = direction of travel (forward only).
%   op 'turn' : arg = target heading; dirs = directions to try in order (+1 CCW, -1 CW,
%               [] = shortest first, then the long way). tol = heading tolerance (deg).
%   op 'dump' / 'hold': arg = struct of extra fields (dump: spot, D); no motion.
%   Each step: type, p, h, dir, tol, poses (swept samples K-by-3), need (filled by route_needs).
ok = true; why = '';
if nargin < 8 || isempty(tol), tol = P.brain.turnTolDeg; end
p0 = st.pose;
switch op
    case 'drive'
        d = hypot(arg(1)-p0(1), arg(2)-p0(2));
        if d < 1e-6, return; end
        h = atan2(arg(2)-p0(2), arg(1)-p0(1));
        if abs(angdiff(h, p0(3))) > deg2rad(P.brain.turnTolDeg) + 1e-9
            error('route_steps:heading', 'drive needs a turn first (%.1f deg off)', rad2deg(angdiff(h, p0(3))));
        end
        n = max(2, ceil(d/0.005) + 1); f = linspace(0, 1, n)';
        poses = [p0(1) + f*(arg(1)-p0(1)), p0(2) + f*(arg(2)-p0(2)), h*ones(n,1)];
        [ok, why] = route_check(poses, piles, L, P);
        if ~ok, why = ['drive: ' why]; return; end
        % long drives are cut into pieces of <= 0.05 m that the robot passes without stopping (it
        % steers to the end point throughout); sections are then released piece by piece
        m = max(1, ceil(d/0.05)); cut = round(linspace(1, n, m+1));
        for j = 1:m
            s = mk('drive', arg, h, 0, tol, poses(cut(j):cut(j+1), :));
            s.pass = j < m;                                  % pass-through piece: no braking
            s.pEnd = poses(cut(j+1), 1:2);
            st.steps(end+1) = s;
        end
        st.pose = [arg(:)', h];
    case 'turn'
        e = angdiff(arg, p0(3));
        if abs(e) < 1e-9, st.pose(3) = arg; return; end
        if nargin < 7 || isempty(dirs), dirs = [sign(e), -sign(e)]; end
        whys = {};
        for dr = dirs
            a = mod(dr*(arg - p0(3)), 2*pi);                % angle swept in direction dr
            n = max(2, ceil(a/deg2rad(3)) + 1);
            th = p0(3) + dr*linspace(0, a, n)';
            poses = [p0(1)*ones(n,1), p0(2)*ones(n,1), th];
            [ok, w] = route_check(poses, piles, L, P);
            if ok
                st.steps(end+1) = mk('turn', p0(1:2), arg, dr, tol, poses);
                st.pose = [p0(1:2), arg];
                return
            end
            whys{end+1} = w; %#ok<AGROW>
        end
        why = ['turn: ' whys{1}];
    case {'dump', 'hold'}
        s = mk(op, p0(1:2), p0(3), 0, tol, p0);
        if isstruct(arg), s.spot = arg.spot; s.D = arg.D; end
        st.steps(end+1) = s;
end
end

function s = mk(type, p, h, dir, tol, poses)
s = struct('type', type, 'p', p(:)', 'h', h, 'dir', dir, 'tol', tol, 'poses', poses, ...
    'need', [], 'spot', 0, 'D', 0, 'pass', false, 'pEnd', p(:)');
end

function e = angdiff(a, b)
e = atan2(sin(a-b), cos(a-b));
end
