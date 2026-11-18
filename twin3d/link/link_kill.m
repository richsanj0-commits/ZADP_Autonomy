function left = link_kill(procs)
%LINK_KILL  Stop every process link_launch started (and any PID the Python side reported), then check by PID.
%   left = link_kill(procs) returns the PIDs still alive afterwards (must be empty). Safe to call twice.
left = [];
if isempty(procs) || ~isfield(procs, 'pids'), return; end
for pid = procs.pids(:)'
    system(sprintf('taskkill /PID %d /T /F >nul 2>&1', pid));
end
t0 = tic;
while true
    left = procs.pids(arrayfun(@alive, procs.pids));
    if isempty(left) || toc(t0) > 5, break; end
    pause(0.2);
end
if isfield(procs, 'objs')
    for i = 1:numel(procs.objs)
        try
            procs.objs{i}.Dispose();
        catch
        end
    end
end
end

function a = alive(pid)
[~, out] = system(sprintf('tasklist /FI "PID eq %d" /NH', pid));
a = ~isempty(regexp(out, ['\<' num2str(pid) '\>'], 'once'));
end
