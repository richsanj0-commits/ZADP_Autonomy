classdef BrainSystem < matlab.System
%BRAINSYSTEM  The central brain as a Simulink MATLAB System block (Simulate using: Interpreted execution).
%   Holds the brain world W (brain_init), calls brain_step once per sample time (P.brain.dt = 0.02 s) and outputs
%   what the 3D view and the displays need. No input ports.
%   Outputs
%     1 pose       3-by-3   true robot poses [x y heading], one row per robot
%     2 mode       3-by-1   0 parked, 1 driving, 2 at the dump stop
%     3 piles      1-by-1   piles placed
%     4 coverage   1-by-1   site covered by piles, % (brain_coverage; recomputed only when a pile lands)
%     5 time       1-by-1   sim time, s
%     6 bed        3-by-1   dump body angle, deg
%     7 truck      3-by-1   truck number carried (0 = none: parked)
%     8 dump       3-by-6   pile being dumped by each robot: [x y r h key frac] (zeros = none), arena m
%     9 pileTable  MaxPiles-by-5  placed piles [x y r h key] (rows of zeros are unused), arena m
%    10 collisions 1-by-1   robot-robot and robot-pile overlap events so far (brain_run's monitors, every step)
%   The 3D view gets everything through these signals, so there is no shared handle or global between the two
%   blocks: View3DSystem only needs its input ports (see simulink/README.md).
    properties (Nontunable)
        Seed = 1                 % brain seed (same as brain_run)
        MaxPiles = 130           % size of the pile table output (>= number of spots)
        SampleTime = 0.02        % s, = P.brain.dt
    end
    properties (Access = private)
        W; P; plan
        cov = 0; nSeen = -1; table
        collisions = 0; hitRR; hitP
    end
    methods (Access = protected)
        function setupImpl(obj)
            root = fileparts(fileparts(mfilename('fullpath')));
            addpath(fullfile(root, 'params'), fullfile(root, 'plan'), fullfile(root, 'brain'), fullfile(root, 'bot'), ...
                fullfile(root, 'world3d'));
            obj.P = twin_params();
            assert(abs(obj.SampleTime - obj.P.brain.dt) < 1e-12, 'BrainSystem: SampleTime must equal P.brain.dt');
            obj.plan = load_plan(fullfile(root, 'plan', 'plan_arena.json'));
            rng(obj.Seed);
            obj.W = brain_init(obj.plan, obj.P, obj.Seed);
            obj.W.useCamera = true;
            obj.cov = 0; obj.nSeen = -1; obj.collisions = 0;
            nb = numel(obj.W.bots);
            obj.hitRR = false(nb); obj.hitP = false(1, nb);
            obj.table = zeros(obj.MaxPiles, 5);
        end

        function [pose, mode, nPiles, coverage, t, bed, truck, dump, tab, collisions] = stepImpl(obj)
            % The outputs are the state at the START of this sample (time = Simulink time); the brain is advanced by
            % one step at the end of the call.
            P = obj.P;
            W = obj.W; nb = numel(W.bots);
            d = brain_dump_state(W, P);
            % monitors on the true poses, every step (as brain_run)
            for i = 1:nb
                for j = i+1:nb
                    ov = false;
                    if hypot(d.pose(i,1) - d.pose(j,1), d.pose(i,2) - d.pose(j,2)) < 0.25
                        ov = rect_pair(d.pose(i,:), d.pose(j,:), P);
                    end
                    obj.collisions = obj.collisions + (ov && ~obj.hitRR(i,j)); obj.hitRR(i,j) = ov;
                end
                ov = false;
                if ~isempty(W.piles), ov = rect_circle_clear(d.pose(i,:), W.piles, W.pileR(:)', P) < 0; end
                obj.collisions = obj.collisions + (ov && ~obj.hitP(i)); obj.hitP(i) = ov;
            end
            n = size(W.piles, 1);
            if n ~= obj.nSeen                       % a pile landed: refresh the pile table and the coverage
                obj.nSeen = n;
                T = zeros(obj.MaxPiles, 5);
                for k = 1:min(n, obj.MaxPiles)
                    pr = W.pileRec(k);
                    T(k,:) = [W.piles(k,:), pr.rArena, pr.hArena, pr.spot];
                end
                obj.table = T;
                obj.cov = 100*brain_coverage(W, obj.plan);
            end
            pose = d.pose; mode = d.mode; nPiles = n; coverage = obj.cov; t = W.t;
            bed = d.bed; truck = d.truck; dump = d.dump; tab = obj.table; collisions = obj.collisions;
            if ~W.done, obj.W = brain_step(W, P, P.brain.dt); end            % a finished brain just holds
        end

        function resetImpl(obj)
            obj.W = brain_init(obj.plan, obj.P, obj.Seed); obj.W.useCamera = true;
            obj.cov = 0; obj.nSeen = -1; obj.collisions = 0;
        end

        % ---- sizes, types, sample time (fixed-size doubles)
        function n = getNumOutputsImpl(~), n = 10; end
        function varargout = getOutputNamesImpl(~)
            varargout = {'pose', 'mode', 'piles', 'coverage', 'time', 'bed', 'truck', 'dump', 'pileTable', 'collisions'};
        end
        function varargout = getOutputSizeImpl(obj)
            varargout = {[3 3], [3 1], [1 1], [1 1], [1 1], [3 1], [3 1], [3 6], [obj.MaxPiles 5], [1 1]};
        end
        function varargout = getOutputDataTypeImpl(~), varargout = repmat({'double'}, 1, 10); end
        function varargout = isOutputComplexImpl(~), varargout = repmat({false}, 1, 10); end
        function varargout = isOutputFixedSizeImpl(~), varargout = repmat({true}, 1, 10); end
        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.SampleTime);
        end
    end
end
