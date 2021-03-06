L1 = 3;
L2 = 4;
c = 1;

[T, phiBase1] = ode45(@odefun, [0, 2], [5; 4; 1], odeset('MaxStep', 1.4));

Px = phiBase1(:, 1);
Py = phiBase1(:, 2);
Pz = phiBase1(:, 3);

% Px = linspace(1, 5, 200).';
% Py = Px .* 0.5 .* sin(Px) + 3;
% Pz = 3 .* sin(2 .* Px) + 2;

a = L1;
b = sqrt(Px .^ 2 + Pz .^ 2 + Py .^ 2); % Only major difference from 2D version
c = L2;

phiBase = acos((a .^ 2 + b .^ 2 - c .^ 2) ./ (2 .* a .* b)) + atan(Pz ./ sqrt(Px .^ 2 + Py .^ 2)); % Base vertical
phiArm =   acos((a .^ 2 + c .^ 2 - b .^ 2) ./ (2 .* a .* c)) + phiBase + pi ./ 2;
thetaBase = atan2(Py, Px); % Base lateral
Angle = table(phiBase, phiArm, thetaBase);

X1 = L1 .* cos(Angle.phiBase) .* cos(Angle.thetaBase);
Y1 = L1 .* cos(Angle.phiBase) .* sin(Angle.thetaBase);
Z1 = L1 .* sin(Angle.phiBase);
%
X2 = X1 + L2 .* sin(-Angle.phiArm) .* cos(Angle.thetaBase); % may later be T4
Y2 = Y1 + L2 .* sin(-Angle.phiArm) .* sin(Angle.thetaBase); % may later be T4
Z2 = Z1 + L2 .* cos(-Angle.phiArm);

Position = table(X1, Y1, Z1, X2, Y2, Z2);

armX = [zeros(length(Position.X2), 1), Position.X1, Position.X2];
armY = [zeros(length(Position.Y2), 1), Position.Y1, Position.Y2];
armZ = [zeros(length(Position.Z2), 1), Position.Z1, Position.Z2];

figure(1);
clf;
hold on;
plot3(phiBase1(:, 1), phiBase1(:, 2), phiBase1(:, 3));
axis equal;
xlabel("X");
ylabel("Y");
zlabel("Z");

figure(2);
clf;
hold on;

plot3(Px, Py, Pz, 'go');
plot3(armX, armY, armZ, 'r-');

plot3(0, 0, 0, 'ko');
plot3(armX(:, [1, 2]).', armY(:, [1, 2]).', armZ(:, [1, 2]).', 'r-');
plot3(armX(:, 2).',      armY(:, 2).',      armZ(:, 2).',      'kx');
plot3(armX(:, [2, 3]).', armY(:, [2, 3]).', armZ(:, [2, 3]).', 'r-');
plot3(armX(:, 3).',      armY(:, 3).',      armZ(:, 3).',      'kx');

axis equal;
xlabel("X");
ylabel("Y");
zlabel("Z");

% figure(2);
% % while 1
%     for ii = 1:length(Position.X1)
%        clf;
%        hold on;
%        plot3(0, 0, 0, 'ko');
%        plot3(armX(ii, [1, 2]), armY(ii, [1, 2]), armZ(ii, [1, 2]), 'r-');
%        plot3(armX(ii, 2),      armY(ii, 2),      armZ(ii, 2),      'ko');
%        plot3(armX(ii, [2, 3]), armY(ii, [2, 3]), armZ(ii, [2, 3]), 'r-');
%        plot3(armX(ii, 3),      armY(ii, 3),      armZ(ii, 3),      'ko');
%
%        plot3(Px, Py, Pz, 'g-');
%
%        axis equal;
%        view([ii .* 180 ./ length(Position.X1), ii .* 90 ./ length(Position.X1)]);
%        xlim([-(L1 + L2), (L1 + L2)]);
%        ylim([-1, L1 + L2]);
%        zlim([-(L1 + L2), (L1 + L2)]);
%        title(sprintf("Base (pan) = %.2f, Base (tilt) 2 = %.2f, Elbow (tilt) 2 = %.2f", ...
%            thetaBase(ii), phiBase(ii), phiArm(ii)))
%        drawnow;
%        pause(0.015);
%     end
% % end

function dPdt = odefun(t, P)

    Px = P(1);
    Py = P(2);
    Pz = P(3);

    L1 = 5;
    L2 = 5;

    vx = -1;
    vy = 0;
    vz = 0;

    a = L1;
    b = sqrt(Px .^ 2 + Pz .^ 2 + Py .^ 2); % Only major difference from 2D version
    c = L2;

    [thetaBase, phiBase, phiArm] = phiVelocity(vx, vy, vz, Px, Py, Pz, L1, L2);

    if imag(Py) ~= 0
        disp("extended too far")
    end

    % check boundary conditions
    out_of_bounds = Px .^ 2 + Pz .^ 2 + Py .^ 2 > (L1 + L2 - 1) .^ 2;
    if out_of_bounds
        disp("out of bounds!")
        dPdt = [0; 0; 0];
        return
    end


    % phiBase = acos((a .^ 2 + b .^ 2 - c .^ 2) ./ (2 .* a .* b)) + atan(Pz ./ sqrt(Px .^ 2 + Py .^ 2)); % Base vertical
    % phiArm =  acos((a .^ 2 + c .^ 2 - b .^ 2) ./ (2 .* a .* c)) + phiBase + pi ./ 2;
    % thetaBase = atan2(Py, Px); % Base lateral

    X1 = L1 .* cos(phiBase) .* cos(thetaBase);
    Y1 = L1 .* cos(phiBase) .* sin(thetaBase);
    Z1 = L1 .* sin(phiBase);
    %
    X2 = X1 + L2 .* sin(-phiArm) .* cos(thetaBase); % may later be T4
    Y2 = Y1 + L2 .* sin(-phiArm) .* sin(thetaBase); % may later be T4
    Z2 = Z1 + L2 .* cos(-phiArm);

    dPdt = [X2; Y2; Z2];
end