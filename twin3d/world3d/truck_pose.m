function truck_pose(ht, pose, bedAngleDeg, truckNo)
%TRUCK_POSE  Move a truck from truck_build to pose [x y heading] (the rear-axle midpoint, like the robot) and
%   tip its dump body bedAngleDeg up (clamped to 0 .. P.truck.bedTiltMaxDeg). Optional truckNo sets the label
%   "T<n>" (0 hides the label: the robot is parked and carries no truck).
%   The truck rides on the apron (1.5 mm), the gate ramp or the raised site pad (4 mm).
if nargin < 3, bedAngleDeg = 0; end
z = interp1([-1 0 0.05 2], [0.0015 0.0015 0.004 0.004], pose(2), 'linear', 0.004);
set(ht.root, 'Matrix', makehgtform('translate', [pose(1) pose(2) z]) * makehgtform('zrotate', pose(3)));
a = min(max(bedAngleDeg, 0), ht.maxDeg);
set(ht.bed, 'Matrix', ht.bedBase * makehgtform('yrotate', -deg2rad(a)));
if nargin >= 4
    if truckNo > 0
        s = sprintf('T%d', truckNo);
        if ~strcmp(ht.label.String, s), ht.label.String = s; end
        ht.label.Visible = 'on';
    else
        ht.label.Visible = 'off';
    end
end
end
