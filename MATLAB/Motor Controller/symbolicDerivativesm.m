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

%% outputs
- (Px/(L1*(Px^2 + Py^2 + Pz^2)^0.5000) - (0.5000*Px*(L1^2 - L2^2 + Px^2 + Py^2 + Pz^2))/(L1*(Px^2 + Py^2 + Pz^2)^1.5000))/(1 - (0.2500*(L1^2 - L2^2 + Px^2 + Py^2 + Pz^2)^2)/(L1^2*(Px^2 + Py^2 + Pz^2)))^0.5000 - (Px*Pz)/((Pz^2/(Px^2 + Py^2) + 1)*(Px^2 + Py^2)^1.5000)
- (Py/(L1*(Px^2 + Py^2 + Pz^2)^0.5000) - (0.5000*Py*(L1^2 - L2^2 + Px^2 + Py^2 + Pz^2))/(L1*(Px^2 + Py^2 + Pz^2)^1.5000))/(1 - (0.2500*(L1^2 - L2^2 + Px^2 + Py^2 + Pz^2)^2)/(L1^2*(Px^2 + Py^2 + Pz^2)))^0.5000 - (Py*Pz)/((Pz^2/(Px^2 + Py^2) + 1)*(Px^2 + Py^2)^1.5000)
1/((Pz^2/(Px^2 + Py^2) + 1)*(Px^2 + Py^2)^0.5000) - (Pz/(L1*(Px^2 + Py^2 + Pz^2)^0.5000) - (0.5000*Pz*(L1^2 - L2^2 + Px^2 + Py^2 + Pz^2))/(L1*(Px^2 + Py^2 + Pz^2)^1.5000))/(1 - (0.2500*(L1^2 - L2^2 + Px^2 + Py^2 + Pz^2)^2)/(L1^2*(Px^2 + Py^2 + Pz^2)))^0.5000