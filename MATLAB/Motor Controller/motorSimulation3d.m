L1 = 3;
L2 = 4;
c = 1;

Px = linspace(1, 5, 200).';
% Py = (Px .* -0.5 + 2);
Py = 3 .* sin(2 .* Px) + 2;
Pz = Px .* 0.5 .* sin(Px) + 3;

% T1 = (asin((Px .^ 2 + Py .^ 2 + L1 .^ 2 - L2 .^ 2) ./ (sqrt(Px .^ 2 + Py .^ 2) .* 2)) - atan(Py ./ Px));

a = L1;
b = sqrt(Px .^ 2 + Py .^ 2 + Pz .^ 2); % Only major difference from 2D version
c = L2;

T1 = -(acos((a .^ 2 + b .^ 2 - c .^ 2) ./ (2 .* a .* b)) + atan(Py ./ sqrt(Px .^ 2 + Pz .^ 2))) + pi ./ 2; % Base vertical
T2 = -acos((-cos(T1) .* L1 + Py) / L2); % Elbow
T3 = acos(Pz ./ (Px .^ 2 + Pz .^ 2)); % Base horizontal
Angle = table(T1, T2, T3);

X1 = L1 .* sin(Angle.T1) .* sin(Angle.T3);
Y1 = L1 .* cos(Angle.T1);
Z1 = L1 .* sin(Angle.T1) .* cos(Angle.T3);

X2 = X1 + L2 .* sin(-Angle.T2) .* sin(Angle.T3); % may later be T4
Y2 = Y1 + L2 .* cos(-Angle.T2);
Z2 = Z1 + L2 .* sin(-Angle.T2) .* cos(Angle.T3); % may later be T4

Position = table(X1, Y1, Z1, X2, Y2, Z2);


% figure(2);
% clf;
% hold on;
armX = [zeros(length(Position.X1), 1), Position.X1, Position.X2];
armY = [zeros(length(Position.Y1), 1), Position.Y1, Position.Y2];
armZ = [zeros(length(Position.Z1), 1), Position.Z1, Position.Z2];

figure(1);
clf;
hold on;
% plot(Position.X1, Position.Y1, 'b-');
plot3(0, 0, 0, 'kx', 'MarkerSize', 20);
% plot(Position.X2, Position.Y2, 'k.');
% plot(Px, Py, 'g-');

N = [1, 0, -1];
R = diag([1; 1; 1]) - 2 .* N.' * N; % https://math.stackexchange.com/questions/693414/reflection-across-the-plane
RP = (R * [Px, Py, Pz].').';

% plot3(RP(:, 1), RP(:, 2), RP(:, 3), 'b--');


plot3(Px, Py, Pz, 'g-');
plot3(armX, armY, armZ, 'r-');

plot3(0, 0, 0, 'ko')
plot3(armX(:, [1, 2]).', armY(:, [1, 2]).', armZ(:, [1, 2]).', 'r-')
plot3(armX(:, 2).', armY(:, 2).', armZ(:, 2).', 'ko')
plot3(armX(:, [2, 3]).', armY(:, [2, 3]).', armZ(:, [2, 3]).', 'r-')
plot3(armX(:, 3).', armY(:, 3).', armZ(:, 3).', 'ko')


axis equal;
xlabel("X");
ylabel("Y");
zlabel("Z");

% figure(2);
% while 1
%     for ii = 1:length(Position.X1)
%        clf;
%        hold on;
%        plot3(0, 0, 0, 'ko')
%        plot3(armX(ii, [1, 2]), armY(ii, [1, 2]), armZ(ii, [1, 2]), 'r-')
%        plot3(armX(ii, 2), armY(ii, 2), armZ(ii, 2), 'ko')
%        plot3(armX(ii, [2, 3]), armY(ii, [2, 3]), armZ(ii, [2, 3]), 'r-')
%        plot3(armX(ii, 3), armY(ii, 3), armZ(ii, 3), 'ko')

%        plot3(Px, Py, Pz, 'g-');
% %        yline(c, '--');

%        axis equal;
%        view(3)
%        xlim([-(L1 + L2), (L1 + L2)]);
%        ylim([-1, L1 + L2]);
%        zlim([-(L1 + L2), (L1 + L2)]);
%        title(sprintf("Theta 1 = %.2f, Theta 2 = %.2f", T1(ii), T2(ii)))
%        drawnow;
%        pause(0.015);
%        [caz, cel]
%     end
% end