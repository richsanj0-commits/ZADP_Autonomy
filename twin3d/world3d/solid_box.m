function p = solid_box(par, x, y, z, col)
%SOLID_BOX  Axis-aligned box as one flat-shaded patch. x, y, z = [min max]. par = parent (axes or hgtransform).
V = [x(1) y(1) z(1); x(2) y(1) z(1); x(2) y(2) z(1); x(1) y(2) z(1); ...
     x(1) y(1) z(2); x(2) y(1) z(2); x(2) y(2) z(2); x(1) y(2) z(2)];
F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
p = patch('Parent', par, 'Vertices', V, 'Faces', F, 'FaceColor', col, 'EdgeColor', 'none', ...
    'FaceLighting', 'flat', 'AmbientStrength', 0.55, 'DiffuseStrength', 0.75, 'SpecularStrength', 0.05);
end
