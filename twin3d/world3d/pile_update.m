function pile_update(hp, r, h)
%PILE_UPDATE  Set the size of a pile from pile_draw to base radius r and height h (arena m). Used for the growth
%   while the truck dumps: call it with the radius and height from pile_growth(frac, rFull, hFull).
%   The apex ring has radius 0 and height h exactly, the base ring radius r exactly (see pile_draw).
nr = hp.nr; s = (0:nr-1)'/(nr - 1);              % 0 base ... 1 apex
R = r*(1 - s) .* hp.rk;                          % ring radius
Z = hp.base + h*s .* hp.zk;
Z(nr, :) = hp.base + h;
Z(1, :) = hp.base;
set(hp.surf, 'XData', hp.c(1) + R.*cos(hp.th), 'YData', hp.c(2) + R.*sin(hp.th), 'ZData', Z);
end
