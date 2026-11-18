function p = solid_wheel(par, c, r, hw, col, n)
%SOLID_WHEEL  Wheel (cylinder, axis along y) centre c = [x y z], radius r, half width hw, n sides.
if nargin < 6, n = 14; end
a = (0:n-1)'*2*pi/n;
V = [c(1) + r*cos(a), (c(2) - hw)*ones(n,1), c(3) + r*sin(a); c(1) + r*cos(a), (c(2) + hw)*ones(n,1), c(3) + r*sin(a)];
F = NaN(n + 2, n);
F(1,:) = 1:n; F(2,:) = n + (1:n);
for i = 1:n
    j = mod(i, n) + 1;
    F(2 + i, 1:4) = [i j n+j n+i];
end
p = patch('Parent', par, 'Vertices', V, 'Faces', F, 'FaceColor', col, 'EdgeColor', 'none', ...
    'FaceLighting', 'flat', 'AmbientStrength', 0.5, 'DiffuseStrength', 0.7, 'SpecularStrength', 0);
end
