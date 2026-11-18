classdef View3DSystem < matlab.System
%VIEW3DSYSTEM  Simulink sink block: the 3D world (world_build) updated from its input signals.
%   Builds the scene in setup (a hidden figure unless Visible = 'on'; tag 'zadp_twin_view') and, every RenderPeriod
%   seconds of sim time, moves the trucks, tips the dump bodies, grows the pile being dumped, adds finished piles
%   and writes the HUD (world_sync, world_hud). Everything it draws comes in through the input ports, wired from
%   BrainSystem's outputs; nothing is shared through a global or a handle, so the block works with any source of
%   the same signals.
%   Inputs: 1 pose (3x3), 2 bed (3x1 deg), 3 truck (3x1), 4 dump (3x6), 5 pileTable (MaxPiles x 5), 6 piles placed,
%           7 coverage (%), 8 sim time, 9 collisions.
%   The figure keeps appdata 'renders' (number of picture updates) and 'tLast'; with PngFile set, the last picture
%   is written to that file when the simulation ends (release).
    properties (Nontunable)
        Visible = 'off'          % 'off' (hidden figure) or 'on'
        RenderPeriod = 0.25      % s of sim time between picture updates
        MaxPiles = 130           % rows of the pile table input
        SampleTime = 0.02        % s, = BrainSystem sample time
        PngFile = ''             % write the last picture here at the end of the run (optional)
        ViewMode = 'orbit'       % world_view mode
    end
    properties (Access = private)
        h; plan; P; lastRender; nSpots
    end
    methods (Access = protected)
        function setupImpl(obj)
            root = fileparts(fileparts(mfilename('fullpath')));
            addpath(fullfile(root, 'params'), fullfile(root, 'plan'), fullfile(root, 'brain'), fullfile(root, 'world3d'));
            old = findall(0, 'Type', 'figure', 'Tag', 'zadp_twin_view');
            close(old);
            obj.P = twin_params();
            obj.plan = load_plan(fullfile(root, 'plan', 'plan_arena.json'));
            obj.h = world_build(obj.plan, obj.P, struct('visible', obj.Visible));
            set(obj.h.fig, 'Tag', 'zadp_twin_view');
            setappdata(obj.h.fig, 'renders', 0); setappdata(obj.h.fig, 'tLast', -1);
            world_view(obj.h, obj.ViewMode);
            obj.lastRender = -Inf; obj.nSpots = numel(obj.plan.trucks(1).spots);
        end

        function stepImpl(obj, pose, bed, truck, dump, tab, nPiles, cov, t, collisions)
            if t - obj.lastRender < obj.RenderPeriod - 1e-9, return; end
            obj.lastRender = t;
            n = min(round(nPiles), size(tab, 1));
            piles = struct('c', cell(1, n), 'r', 0, 'h', 0, 'frac', 1, 'key', 0);
            for k = 1:n
                piles(k) = struct('c', tab(k,1:2), 'r', tab(k,3), 'h', tab(k,4), 'frac', 1, 'key', tab(k,5));
            end
            for i = find(dump(:,6) > 0)'
                piles(end+1) = struct('c', dump(i,1:2), 'r', dump(i,3), 'h', dump(i,4), 'frac', dump(i,6), ...
                    'key', dump(i,5)); %#ok<AGROW>
            end
            world_sync(obj.h, struct('pose', pose, 'bed', bed(:)', 'truck', truck(:)', 'piles', piles));
            world_hud(obj.h, t, max(truck), n, obj.nSpots, cov, collisions, 'Simulink: zadp\_twin');
            drawnow limitrate;
            setappdata(obj.h.fig, 'renders', getappdata(obj.h.fig, 'renders') + 1);
            setappdata(obj.h.fig, 'tLast', t);
        end

        function releaseImpl(obj)
            if ~isempty(obj.PngFile) && ~isempty(obj.h) && isgraphics(obj.h.fig)
                drawnow;
                fr = getframe(obj.h.fig);
                imwrite(fr.cdata, obj.PngFile);
            end
        end

        function n = getNumInputsImpl(~), n = 9; end
        function varargout = getInputNamesImpl(~)
            varargout = {'pose', 'bed', 'truck', 'dump', 'pileTable', 'piles', 'coverage', 'time', 'collisions'};
        end
        function n = getNumOutputsImpl(~), n = 0; end
        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.SampleTime);
        end
    end
end
