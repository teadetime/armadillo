syms a b c Px Py Pz L1 L2 phiBase phiArm thetaBase

a = L1;
b = sqrt(Px .^ 2 + Pz .^ 2 + Py .^ 2); % Only major difference from 2D version
c = L2;

phiBase = acos((a .^ 2 + b .^ 2 - c .^ 2) ./ (2 .* a .* b)) + atan(Pz ./ sqrt(Px .^ 2 + Py .^ 2)); % Base vertical
phiArm =   acos((a .^ 2 + c .^ 2 - b .^ 2) ./ (2 .* a .* c)) + phiBase + pi ./ 2;
thetaBase = atan2(Py, Px); % Base lateral

diff(phiBase, Px)
diff(phiBase, Py)
diff(phiBase, Pz)