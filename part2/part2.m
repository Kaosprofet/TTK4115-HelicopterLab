load ../part1/part1
addpath(data)

A = [0 1 0; 0 0 0; 0 0 0];
B = [0 0; 0 K_1; K_2 0];
C = [1 0 0; 0 0 1];

Q = diag([150 30 150]);
%Q = diag([100 30 100]);
R = diag([1 10]);

K = lqr(A,B,Q,R);
F = (C*(B*K-A)^(-1)*B)^(-1);
%% Plot task 1
% load('part2-1')
% figure('Color', 'white', 'Name', 'Step responce to equilibrium point')
% 
% plot(simout.time, simout.signals.values(:,3))
% hold on
% plot(simout.time, simout.signals.values(:,5)-0.5)
% 
% grid
% title('Step responce to equilibrium point')
% xlabel('t/[s]')
% ylabel('\theta/[rad]')
% legend('p - Pitch', 'e - Elevation', 'Location', 'southeast')
% 
% figure('Color', 'white', 'Name', 'Step velocity responce to equilibrium point')
% plot(simout.time, simout.signals.values(:,4))
% hold on
% plot(simout.time, simout.signals.values(:,6))
% 
% grid
% title('Step velocity responce to equilibrium point')
% xlabel('t/[s]')
% ylabel('\theta/[rad]')
% legend('dot p - Pitch rate', 'dot e - Elevation rate', 'Location', 'southeast')
%% Task2
%clc

A1 = [A zeros(3,2); C zeros(2,2)];
B1 = [B; zeros(2,2)];

Q = diag([100 30 100 20 50]);
R = diag([0.1 1]);

K = lqr(A1,B1,Q,R);

K1 = K(:,1:3);

F = (C*(B*K1-A)^(-1)*B)^(-1);

%% Plot task 2 
% close all
% clc 
% 
% load('part2-2-1')
% figure('Color', 'white', 'Name', 'Step responce to equilibrium point')
% 
% plot(simout.time, simout.signals.values(:,3))
% hold on
% plot(simout.time, simout.signals.values(:,5)-0.5)
% 
% grid
% title('Step responce to equilibrium point')
% xlabel('t/[s]')
% ylabel('\theta/[rad]')
% legend('p - Pitch', 'e - Elevation', 'Location', 'southeast')
% 
% figure('Color', 'white', 'Name', 'Step velocity responce to equilibrium point')
% plot(simout.time, simout.signals.values(:,4))
% hold on
% plot(simout.time, simout.signals.values(:,6))
% 
% grid
% title('Step velocity responce to equilibrium point')
% xlabel('t/[s]')
% ylabel('\theta/[rad]')
% legend('dot p - Pitch rate', 'dot e - Elevation rate', 'Location', 'southeast')
% 
% 
% 
% 
% load('part2-2_startup_response')
% figure('Color', 'white', 'Name', 'Start-up step responce to equilibrium point')
% plot(simout.time, simout.signals.values(:,3))
% hold on
% plot(simout.time, simout.signals.values(:,5)-0.5)
% 
% grid
% title('Start-up step responce to equilibrium point')
% xlabel('t/[s]')
% ylabel('\theta/[rad]')
% legend('p - Pitch', 'e - Elevation', 'Location', 'southeast')
% 
% figure('Color', 'white', 'Name', 'Start-up step velocity responce to equilibrium point')
% plot(simout.time, simout.signals.values(:,4))
% hold on
% plot(simout.time, simout.signals.values(:,6))
% 
% grid
% title('Start-up step velocity responce to equilibrium point')
% xlabel('t/[s]')
% ylabel('\theta/[rad]')
% legend('dot p - Pitch rate', 'dot e - Elevation rate', 'Location', 'southeast')
% 
% 
% 
% load('part2-2_pitch_response')
% figure('Color', 'white', 'Name', 'Pitch response')
% plot(simout.time, simout.signals.values(:,3))
% hold on
% plot(simout.time, simout.signals.values(:,5)-0.5)
% 
% grid
% title('Pitch response')
% xlabel('t/[s]')
% ylabel('\theta/[rad]')
% legend('p - Pitch', 'e - Elevation', 'Location', 'southeast')
% 
% figure('Color', 'white', 'Name', 'Pitch velocity response')
% plot(simout.time, simout.signals.values(:,4))
% hold on
% plot(simout.time, simout.signals.values(:,6))
% 
% grid
% title('Pitch velocity response')
% xlabel('t/[s]')
% ylabel('\theta/[rad]')
% legend('dot p - Pitch rate', 'dot e - Elevation rate', 'Location', 'southeast')
% 
% load('part2-2_pitch_response2')
% figure('Color', 'white', 'Name', 'Pitch response2')
% plot(simout.time, simout.signals.values(:,3))
% hold on
% plot(simout.time, simout.signals.values(:,5)-0.5)
% 
% grid
% title('Pitch response2')
% xlabel('t/[s]')
% ylabel('\theta/[rad]')
% legend('p - Pitch', 'e - Elevation', 'Location', 'southeast')
% 
% figure('Color', 'white', 'Name', 'Pitch velocity response2')
% plot(simout.time, simout.signals.values(:,4))
% hold on
% plot(simout.time, simout.signals.values(:,6))
% 
% grid
% title('Pitch velocity response2')
% xlabel('t/[s]')
% ylabel('\theta/[rad]')
% legend('dot p - Pitch rate', 'dot e - Elevation rate', 'Location', 'southeast')