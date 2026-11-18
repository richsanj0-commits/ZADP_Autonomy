function id = brain_sections(L, x, y)
%BRAIN_SECTIONS  Section index (see brain_layout) of each point (x, y); same size as x. 0 = none.
%   Every point of the site, the gate mouth and the apron belongs to exactly one section.
id = zeros(size(x));
spine = x >= L.xSpL & x <= L.xSpR;
inSide = x < L.xMid;
zoneY = y >= L.yEntry;                                   % zones start at the entry line
id(zoneY & x < L.xSpL) = 1;
id(zoneY & x > L.xSpR) = 2;
lowSp = spine & y >= 0 & y < L.yJ1;
id(lowSp & inSide) = 3; id(lowSp & ~inSide) = 4;
% junction box: the whole spine width around the branch roads and L.jSide into each branch mouth
jx = y >= L.yJ1 & y < L.yJ2 & x >= L.xSpL - L.jSide & x <= L.xSpR + L.jSide;
id(jx) = 5;
id(spine & y >= L.yJ2) = 6;
% apron top band (and the strip beside the gate mouth below the entry line)
top = (y >= L.yBand & y < 0) | (y >= 0 & ~zoneY & ~spine);
id(top & inSide) = 3;
id(top & ~inSide & x <= L.xSpR) = 4;
id(top & x > L.xSpR) = 7;
bot = y < L.yBand;
id(bot & x < L.xQM(1)) = 8;
id(bot & x >= L.xQM(1) & x < L.xQM(2)) = 9;
id(bot & x >= L.xQM(2) & x < L.xQ3) = 10;
id(bot & x >= L.xQ3) = 11;
end
