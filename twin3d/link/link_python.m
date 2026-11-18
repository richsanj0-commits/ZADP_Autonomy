function [exe, hasPaho, ver] = link_python(opts)
%LINK_PYTHON  Find the Python interpreter for the link and say whether paho-mqtt is importable.
%   [exe, hasPaho, ver] = link_python(opts)   opts.python = command or full path (default 'python').
%   exe is the full path from sys.executable, or '' when Python is missing (callers then SKIP).
%   Nothing is installed or changed; loopback mode needs only the Python standard library.
if nargin < 1, opts = struct(); end
cmd = 'python';
if isfield(opts, 'python') && ~isempty(opts.python), cmd = opts.python; end
exe = ''; hasPaho = false; ver = '';
[st, out] = system(['"' cmd '" -c "import sys; print(sys.executable); print(sys.version.split()[0])"']);
if st ~= 0, return; end
lines = strsplit(strtrim(out), newline);
if numel(lines) < 2 || ~isfile(strtrim(lines{1})), return; end
exe = strtrim(lines{1}); ver = strtrim(lines{2});
[st, ~] = system(['"' exe '" -c "import paho.mqtt.client"']);
hasPaho = (st == 0);
end
