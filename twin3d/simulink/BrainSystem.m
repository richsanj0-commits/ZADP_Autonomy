classdef BrainSystem < matlab.System
%BRAINSYSTEM  The central brain as a Simulink MATLAB System block (Simulate using: Interpreted execution).
%   Holds the brain world W (brain_init), calls brain_step once per sample time (P.brain.dt = 0.02 s) and outputs
%   what the 3D view and the displays need. No input ports.
%   Outputs (nb = P.bot.count robots)
%     1 pose       nb-by-3  true robot poses [x y heading], one row per robot
%     2 mode       nb-by-1  0 parked, 1 driving, 2 at the dump stop
%     3 piles      1-by-1   piles placed
%     4 coverage   1-by-1   site covered by piles, % (brain_coverage; recomputed only when a pile lands)
%     5 time       1-by-1   sim time, s
%     6 bed        nb-by-1  dump body angle, deg
%     7 truck      nb-by-1  truck number carried (0 = none: parked)
%     8 dump       nb-by-6  pile being dumped by each robot: [x y r h key frac] (zeros = none), arena m
%     9 pileTable  MaxPiles-by-5  placed piles [x y r h key] (rows of zeros are unused), arena m
%    10 collisions 1-by-1   robot-robot + robot-pile + off-site events so far (brain_monitor, every step, as brain_run)
%    11 trucks     1-by-1   trucks dispatched so far (W.truckNo; every departure from P1 is a new truck)
%    12 done       1-by-1   1 once the run is finished (Rule 8) or deadlocked: the model's Stop block ends the run
%   The 3D view gets everything through these signals, so there is no shared handle or global between the two
%   blocks: View3DSystem only needs its input ports (see simulink/README.md).
%   Repeatable: the brain uses its own random stream (W.rs, seeded with Seed in brain_init), so a model run, a
%   re-run (Fast Restart included) and brain_run with the same seed are the same run.
    properties (Nontunable)
        Seed = 1                 % brain seed (same as brain_run)
        PlanFile = ''            % plan JSON; '' = plan/plan_arena.json (maps/plan_<name>.json for the other maps)
        MaxPiles = 130           % size of the pile table output (>= number of spots, checked in setup)
        SampleTime = 0.02        % s, = P.brain.dt
    end
    properties (Access = private)
        W; P; plan
        cov = 0; nSeen = -1; table
        M
    end
    methods
        function obj = BrainSystem(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end
    methods (Access = protected)
        function setupImpl(obj)
            root = fileparts(fileparts(mfilename('fullpath')));
            addpath(fullfile(root, 'params'), fullfile(root, 'plan'), fullfile(root, 'brain'), fullfile(root, 'bot'), ...
                fullfile(root, 'world3d'));
            obj.P = twin_params();
            assert(abs(obj.SampleTime - obj.P.brain.dt) < 1e-12, 'BrainSystem: SampleTime must equal P.brain.dt');
            obj.plan = load_plan(BrainSystem.planPath(obj.PlanFile));
            nSp = numel(obj.plan.trucks(1).spots);
            assert(nSp <= obj.MaxPiles, 'BrainSystem: the plan has %d spots, more than MaxPiles = %d: raise MaxPiles', nSp, obj.MaxPiles);
            obj.start();
        end

        function [pose, mode, nPiles, coverage, t, bed, truck, dump, tab, collisions, trucks, done] = stepImpl(obj)
            % The outputs are the state at the START of this sample (time = Simulink time); the brain is advanced by
            % one step at the end of the call.
            P = obj.P;
            W = obj.W;
            d = brain_dump_state(W, P);
            n = size(W.piles, 1);
            if n ~= obj.nSeen                       % a pile landed: refresh the pile table and the coverage
                obj.nSeen = n;
                T = zeros(obj.MaxPiles, 5);
                for k = 1:n
                    pr = W.pileRec(k);
                    T(k,:) = [W.piles(k,:), pr.rArena, pr.hArena, pr.spot];
                end
                obj.table = T;
                obj.cov = 100*brain_coverage(W, obj.plan);
            end
            pose = d.pose; mode = d.mode; nPiles = n; coverage = obj.cov; t = W.t;
            bed = d.bed; truck = d.truck; dump = d.dump; tab = obj.table;
            collisions = brain_monitor_collisions(obj.M);
            trucks = W.truckNo; done = double(W.done || W.deadlock);
            if ~(W.done || W.deadlock)                   % a finished brain just holds
                obj.W = brain_step(W, P, P.brain.dt);
                obj.M = brain_monitor(obj.M, obj.W, P);   % monitors on the true poses, every step (as brain_run)
            end
        end

        function resetImpl(obj)
            obj.start();
        end

        % ---- sizes, types, sample time (fixed-size doubles)
        function n = getNumOutputsImpl(~), n = 12; end
        function varargout = getOutputNamesImpl(~)
            varargout = {'pose', 'mode', 'piles', 'coverage', 'time', 'bed', 'truck', 'dump', 'pileTable', 'collisions', ...
                'trucks', 'done'};
        end
        function varargout = getOutputSizeImpl(obj)
            nb = BrainSystem.botCount();
            varargout = {[nb 3], [nb 1], [1 1], [1 1], [1 1], [nb 1], [nb 1], [nb 6], [obj.MaxPiles 5], [1 1], [1 1], [1 1]};
        end
        function varargout = getOutputDataTypeImpl(~), varargout = repmat({'double'}, 1, 12); end
        function varargout = isOutputComplexImpl(~), varargout = repmat({false}, 1, 12); end
        function varargout = isOutputFixedSizeImpl(~), varargout = repmat({true}, 1, 12); end
        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.SampleTime);
        end
    end
    methods (Access = private)
        function start(obj)
            % a fresh world: same seed -> same run (brain_init seeds the world's own stream)
            obj.W = brain_init(obj.plan, obj.P, obj.Seed);
            obj.W.useCamera = true;
            obj.cov = 0; obj.nSeen = -1;
            obj.table = zeros(obj.MaxPiles, 5);
            obj.M = brain_monitor([], obj.W, obj.P);
        end
    end
    methods (Static)
        function f = planPath(planFile)
            % '' -> the reference plan; a bare name is looked up in plan/ and maps/
            root = fileparts(fileparts(mfilename('fullpath')));
            if isempty(planFile), f = fullfile(root, 'plan', 'plan_arena.json'); return; end
            f = planFile;
            if ~isfile(f)
                for d = {'plan', 'maps'}
                    if isfile(fullfile(root, d{1}, planFile)), f = fullfile(root, d{1}, planFile); return; end
                end
                error('BrainSystem: plan file %s not found', planFile);
            end
        end
        function nb = botCount()
            root = fileparts(fileparts(mfilename('fullpath')));
            addpath(fullfile(root, 'params'));
            P = twin_params(); nb = P.bot.count;
        end
    end
end
