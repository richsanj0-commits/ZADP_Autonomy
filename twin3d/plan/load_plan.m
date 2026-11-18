function plan = load_plan(file)
% LOAD_PLAN  Read a plan.json exported by zadp_game.html (exportPlan) and normalise the shapes jsondecode returns.
%   plan.boundary N-by-2; plan.safe cell of N-by-2; plan.zones(i).poly N-by-2, .branch cell of N-by-2;
%   plan.spots.(field) struct array (x,y,zone,order numeric; kind char); plan.trucks(i).spots = same, in truck order.
plan = jsondecode(fileread(file));
if ~isfield(plan, 'version') || ~isequal(plan.version, 1)
    error('load_plan:version', 'Unsupported plan version (need 1) in %s', file);
end
plan.boundary = toPoly(plan.boundary);
plan.safe = toPolys(plan.safe);
plan.spine.poly = toPoly(plan.spine.poly);
plan.spine.inLine = toPoly(plan.spine.inLine);
plan.spine.outLine = toPoly(plan.spine.outLine);
plan.entry.pt = plan.entry.pt(:)'; plan.entry.n = plan.entry.n(:)'; plan.entry.t = plan.entry.t(:)';
zs = plan.zones; plan.zones = struct('n', {}, 'poly', {}, 'branch', {}, 'branchDepth', {}, 'depth', {});
for i = 1:numel(zs)
    z = zs(i); if iscell(zs), z = zs{i}; end
    plan.zones(i).n = z.n; plan.zones(i).poly = toPoly(z.poly);
    plan.zones(i).branch = toPolys(z.branch);
    bd = z.branchDepth; if isempty(bd), bd = NaN; end
    plan.zones(i).branchDepth = bd; plan.zones(i).depth = z.depth;
end
tr = plan.trucks; if iscell(tr), tr = [tr{:}]; end
plan.trucks = tr(:)';
fn = fieldnames(plan.spots);
for i = 1:numel(fn)
    plan.spots.(fn{i}) = toSpots(plan.spots.(fn{i}));
end
for i = 1:numel(plan.trucks)
    if i <= numel(fn), plan.trucks(i).spots = plan.spots.(fn{i}); else, plan.trucks(i).spots = toSpots([]); end
end
end

function s = toSpots(a)
s = struct('x', {}, 'y', {}, 'zone', {}, 'kind', {}, 'order', {});
if isempty(a), return; end
if iscell(a), a = [a{:}]; end
for i = 1:numel(a)
    s(i).x = double(a(i).x); s(i).y = double(a(i).y); s(i).zone = double(a(i).zone);
    s(i).kind = char(a(i).kind); s(i).order = double(a(i).order);
end
s = s(:)';
end

function P = toPoly(m)
% any numeric array holding one polygon -> N-by-2
if iscell(m), m = cell2mat(cellfun(@(r) r(:)', m(:), 'UniformOutput', false)); end
m = squeeze(m);
if ndims(m) > 2, m = reshape(m, [], 2); end
if size(m, 2) ~= 2 && size(m, 1) == 2, m = m'; end
P = double(m);
end

function C = toPolys(a)
% jsondecode gives [] , a K-by-N-by-2 array (equal sizes) or a cell of matrices -> cell of N-by-2
C = {};
if isempty(a), return; end
if iscell(a)
    for i = 1:numel(a), C{end+1} = toPoly(a{i}); end %#ok<AGROW>
elseif ndims(a) == 3
    for i = 1:size(a, 1), C{end+1} = toPoly(a(i, :, :)); end %#ok<AGROW>
else
    C = {toPoly(a)};
end
end
