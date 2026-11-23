function [up, isLocal] = link_broker_status(P)
%LINK_BROKER_STATUS  Does an MQTT broker answer on P.mqtt.broker:P.mqtt.port, and is that host this PC?
%   [up, isLocal] = link_broker_status(P)
%   up      true when a TCP connection to the broker port opens within 2 s (a real broker or mini_broker.py)
%   isLocal true when P.mqtt.broker is localhost / a loopback address / an address of one of this PC's interfaces
%           (then link_launch may start py/mini_broker.py itself when nothing answers)
%   Base MATLAB only (tcpclient, Java networking); nothing is changed.
host = char(P.mqtt.broker);
up = false;
try
    t = tcpclient(host, P.mqtt.port, 'ConnectTimeout', 2);
    delete(t);
    up = true;
catch
end
isLocal = any(strcmpi(host, {'localhost', '127.0.0.1', '::1', '0.0.0.0'})) || startsWith(host, '127.');
if ~isLocal
    try
        % rule: the host is this PC when its address belongs to one of this PC's network interfaces
        a = java.net.InetAddress.getByName(host);
        isLocal = a.isLoopbackAddress() || ~isempty(java.net.NetworkInterface.getByInetAddress(a));
    catch
        isLocal = false;
    end
end
end
