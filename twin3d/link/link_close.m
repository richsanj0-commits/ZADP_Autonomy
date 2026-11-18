function link_close(L, stopRobots)
%LINK_CLOSE  Close the link. By default every robot gets a stop command first (safe state);
%   link_close(L, false) closes without it. Never errors (it runs from cleanup code).
if nargin < 2, stopRobots = true; end
try
    if stopRobots, link_stop_all(L); pause(0.05); end
catch
end
try
    delete(L.tcp);
catch
end
end
