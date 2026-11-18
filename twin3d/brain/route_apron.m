function steps = route_apron(L, P, from, to)
%ROUTE_APRON  Queue advance inside the apron: slot 'from' -> slot 'to' (to = from-1), driving west
%   along the queue row (robots park heading west, so no turn is needed).
st = route_new(L.slot(from,:));
[st, ok, why] = route_steps(st, 'drive', L.slot(to,1:2), zeros(0,2), L, P);
assert(ok, 'route_apron: %s', why);
st = route_steps(st, 'hold', [], zeros(0,2), L, P);
steps = route_needs(st.steps, L, P);
end
