classdef View3DSystem < matlab.System
%VIEW3DSYSTEM  Simulink sink block: the 3D world (world_build) updated from its input signals.
%   Builds the scene in setup (tag 'zadp_twin_view'; hidden figure unless Visible = 'on') and, every RenderPeriod
%   seconds of sim time, moves the trucks, tips the dump bodies, grows the pile being dumped, adds finished piles
%   and writes the HUD (world_sync, world_hud). Everything it draws comes in through the input ports, wired from
%   BrainSystem's outputs; nothing is shared through a global or a handle, so the block works with any source of
%   the same signals.
%   Inputs: 1 pose (nb x 3), 2 bed (nb x 1 deg), 3 truck (nb x 1), 4 dump (nb x 6), 5 pileTable (MaxPiles x 5),
%           6 piles placed, 7 coverage (%), 8 sim time, 9 collisions, 10 trucks dispatched.
%   Closing the figure during a run is fine: the block stops drawing and the simulation goes on.
%   End of run (release): with PngFile set the last picture is written there; a hidden figure is closed, a
%   visible one stays open for the user. The number of picture updates of the last run is kept in the root
%   appdata 'zadp_twin_renders' (run_twin_model reports it).
    properties (Nontunable)
        Visible = 'off'          % 'off' (hidden figure) or 'on'
        RenderPeriod = 0.25      % s of sim time between picture updates
        PlanFile = ''            % plan JSON, as BrainSystem.PlanFile ('' = plan/plan_arena.json)
        SampleTime = 0.02        % s, = BrainSystem sample time
        PngFile = ''             % write the last picture here at the end of the run (optional)
        ViewMode = 'orbit'       % world_view mode
    end
    properties (Access = private)
        h; plan; P; lastRender; nSpots; renders = 0
    end
    methods
        function obj = View3DSystem(varargin)
            % name-value construction from MATLAB (tests): View3DSystem('Visible', 'off', 'RenderPeriod', 0)
            setProperties(obj, nargin, varargin{:});
        end
    end
    methods (Access = protected)
        function setupImpl(obj)
            root = fileparts(fileparts(mfilename('fullpath')));
            addpath(fullfile(root, 'params'), fullfile(root, 'plan'), fullfile(root, 'brain'), fullfile(root, 'world3d'), ...
                fullfile(root, 'simulink'));
            % a HIDDEN figure left by an earlier run that ended in an error (release not called) is closed; a
            % visible one belongs to the user and is kept
            old = findall(0, 'Type', 'figure', 'Tag', 'zadp_twin_view', 'Visible', 'off');
            close(old);
            obj.P = twin_params();
            obj.plan = load_plan(BrainSystem.planPath(obj.PlanFile));
            obj.h = world_build(obj.plan, obj.P, struct('visible', obj.Visible));
            set(obj.h.fig, 'Tag', 'zadp_twin_view');
            world_view(obj.h, obj.ViewMode);
            obj.nSpots = numel(obj.plan.trucks(1).spots);
            obj.resetImpl();
        end

        function resetImpl(obj)
            obj.lastRender = -Inf; obj.renders = 0;
        end

        function stepImpl(obj, pose, bed, truck, dump, tab, nPiles, cov, t, collisions, trucks)
            if t - obj.lastRender < obj.RenderPeriod - 1e-9, return; end
            if isempty(obj.h) || ~isgraphics(obj.h.fig), return; end     % figure closed by the user: just stop drawing
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
            try
                world_sync(obj.h, struct('pose', pose, 'bed', bed(:)', 'truck', truck(:)', 'piles', piles));
                world_hud(obj.h, t, trucks, n, obj.nSpots, cov, collisions, 'Simulink: zadp\_twin');
                drawnow limitrate;
            catch err
                % the figure can be closed between the check above and the drawing (drawnow runs callbacks)
                if isgraphics(obj.h.fig), rethrow(err); end
                return
            end
            obj.renders = obj.renders + 1;
            setappdata(0, 'zadp_twin_renders', obj.renders);
        end

        function releaseImpl(obj)
            setappdata(0, 'zadp_twin_renders', obj.renders);
            if isempty(obj.h) || ~isgraphics(obj.h.fig), return; end
            if ~isempty(obj.PngFile)
                drawnow;
                fr = getframe(obj.h.fig);
                imwrite(fr.cdata, obj.PngFile);
            end
            if strcmp(get(obj.h.fig, 'Visible'), 'off'), close(obj.h.fig); end
        end

        function n = getNumInputsImpl(~), n = 10; end
        function varargout = getInputNamesImpl(~)
            varargout = {'pose', 'bed', 'truck', 'dump', 'pileTable', 'piles', 'coverage', 'time', 'collisions', 'trucks'};
        end
        function n = getNumOutputsImpl(~), n = 0; end
        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.SampleTime);
        end
    end
end
