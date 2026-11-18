function st = route_new(pose)
%ROUTE_NEW  Empty route under construction starting at pose [x y th] (see route_steps).
st.pose = pose(:)';
st.steps = struct('type', {}, 'p', {}, 'h', {}, 'dir', {}, 'tol', {}, 'poses', {}, ...
    'need', {}, 'spot', {}, 'D', {}, 'pass', {}, 'pEnd', {});
end
