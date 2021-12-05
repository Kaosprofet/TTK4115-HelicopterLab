load ../part2/part2
addpath(data)

%IMU
PORT = 4;

A0 = [0 1 0; 0 0 0; 0 0 0];
B0 = [0 0; 0 K_1; K_2 0];
C0 = [1 0 0; 0 0 1];
A01 = [A0 zeros(3,2); C0 zeros(2,2)];
B01 = [B0; zeros(2,2)];

Q = diag([100 30 100 20 50]);
R = diag([0.1 1]);

K = lqr(A01,B01,Q,R);
K1 = K(:,1:3);
F = (C0*(B0*K1-A0)^(-1)*B0)^(-1);

% Task 1 noise
load('part4_1_noise_stationary');
coVarianceStationary = cov(fullYIMU.signals.values);

%covariance while in equilibrium
load('part4_1_noise_equilibrium');
Rd = cov(fullYIMU.signals.values(6000:12000,:));
Qd = [0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];
Qd = 0.00001*eye(6);

Ts = 0.002; %Sampletime
% Full system matrices
Asys = [0 1 0 0 0 0; 0 0 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 0; 0 0 0 0 0 1; K_3 0 0 0 0 0];
Bsys = [0 0; 0 K_3; K_2 0; 0 0; 0 0; 0 0];
Csys = eye(6);

x_hat = ones(6,1);
P = eye(6);
x_hatPriori0 = [0, 0, -0.5, 0, 0, 0]';
P_priori0 = cov(x_hatPriori0)*eye(6);

% Continous state space system 
conSys = ss(Asys,Bsys,Csys,0);
% Discrete state space system
discSys = c2d(conSys, Ts, 'zoh');

% Discrete system matrices
Ad = discSys.A;
Bd = discSys.B;
Cd = discSys.C;

data = struct('Ad', Ad, 'Bd', Bd, 'Cd', Cd, 'x_hatPriori0', x_hatPriori0, 'P_priori0', P_priori0, 'Rd', Rd , 'Qd', Qd);

%% Noise plots
close all
load('part4_1_noise_stationary');
figure('Color', 'white', 'Name', 'Noise - Stationary position')
plot(fullYIMU.time,fullYIMU.signals.values)

title('Noise - Stationary position')
xlabel('t/[s]')
grid

figure('Color', 'white', 'Name', 'Noise - Equilibrium position')
load('part4_1_noise_equilibrium');
plot(fullYIMU.time(6000:12000),fullYIMU.signals.values(6000:12000,:))
title('Noise - Equilibrium position')
xlabel('t/[s]')
grid

%% Q testing
close all

load('part4_xHat_Qinf.mat')
figure('Color', 'white', 'Name', 'Kalman filter output - Q=inf')
plot(kalmanX_hat.time, kalmanX_hat.signals.values);
title('Kalman filter output - Q=inf')
legend('p', 'p_{dot} ', 'e', 'e_{dot}', 'lambda', 'lambda_{dot}', 'Location', 'southeast')
xlabel('t/[s]')
grid

load('part4_xHat_Qeye.mat')
figure('Color', 'white', 'Name', 'Kalman filter output - Q=I')
plot(kalmanX_hat.time, kalmanX_hat.signals.values);
title('Kalman filter output - Q=I')
legend('p', 'p_{dot} ', 'e', 'e_{dot}', 'lambda', 'lambda_{dot}', 'Location', 'southeast')
xlabel('t/[s]')
grid

load('part4_xHat_Q0.01.mat')
figure('Color', 'white', 'Name', 'Kalman filter output - Q=0.01')
plot(kalmanX_hat.time, kalmanX_hat.signals.values);
title('Kalman filter output - Q=0.01')
legend('p', 'p_{dot} ', 'e', 'e_{dot}', 'lambda', 'lambda_{dot}', 'Location', 'southeast')
xlabel('t/[s]')
grid

load('part4_xHat_Q0.001.mat')
figure('Color', 'white', 'Name', 'Kalman filter output - Q=0.001')
plot(kalmanX_hat.time, kalmanX_hat.signals.values);
title('Kalman filter output - Q=0.001')
legend('p', 'p_{dot} ', 'e', 'e_{dot}', 'lambda', 'lambda_{dot}', 'Location', 'southeast')
xlabel('t/[s]')
grid

load('part4_xHat_Q0.0001.mat')
figure('Color', 'white', 'Name', 'Kalman filter output - Q=0.0001')
plot(kalmanX_hat.time, kalmanX_hat.signals.values);
title('Kalman filter output - Q=0.0001')
legend('p', 'p_{dot} ', 'e', 'e_{dot}', 'lambda', 'lambda_{dot}', 'Location', 'southeast')
xlabel('t/[s]')
grid

load('part4_xHat_Q0.00001.mat')
figure('Color', 'white', 'Name', 'Kalman filter output - Q=0.00001')
plot(kalmanX_hat.time, kalmanX_hat.signals.values);
title('Kalman filter output - Q=0.00001')
legend('p', 'p_{dot} ', 'e', 'e_{dot}', 'lambda', 'lambda_{dot}', 'Location', 'southeast')
xlabel('t/[s]')
grid

load('part4_xHat_Q0.000001.mat')
figure('Color', 'white', 'Name', 'Kalman filter output - Q=0.000001')
plot(kalmanX_hat.time, kalmanX_hat.signals.values);
title('Kalman filter output - Q=0.000001')
legend('p', 'p_{dot} ', 'e', 'e_{dot}', 'lambda', 'lambda_{dot}', 'Location', 'southeast')
xlabel('t/[s]')
grid

load('part4_xHat_Q0.mat')
figure('Color', 'white', 'Name', 'Kalman filter output - Q=0')
plot(kalmanX_hat.time, kalmanX_hat.signals.values);
title('Kalman filter output - Q=0')
legend('p', 'p_{dot} ', 'e', 'e_{dot}', 'lambda', 'lambda_{dot}', 'Location', 'southeast')
xlabel('t/[s]')
grid

%%
close all

load('part4_xHat_disconnectTest_estimator.mat')
figure('Color', 'white', 'Name', 'Lurenberg observer output - Disconnect test')
plot(Estimator_xhat.time, Estimator_xhat.signals.values);
title("Lurenberg observer output - Disconnect test")
legend('p', 'p_{dot} ', 'e', 'e_{dot}', 'lambda_{dot}', 'Location', 'southeast')
ylim([-0.6 0.3])
grid

load('part4_xHat_disconnectTest.mat');
figure('Color', 'white', 'Name', 'Kalman filter output - Disconnect test')
plot(kalmanX_hat.time, kalmanX_hat.signals.values);
title("Kalman filter output - Disconnect test")
legend('p', 'p_{dot} ', 'e', 'e_{dot}', 'lambda', 'lambda_{dot}', 'Location', 'southwest')
ylim([-0.6 0.3])
grid

load('part4_y_disconnectTest.mat');
figure('Color', 'white', 'Name', 'IMU output - Disconnect test')
plot(fullYIMU.time, fullYIMU.signals.values);
title('IMU output - Disconnect test')
legend('p', 'p_{dot} ', 'e', 'e_{dot}', 'lambda', 'lambda_{dot}', 'Location', 'southeast')
xlim([0,14])
grid

load('part4_pHat_disconnectTest.mat');
figure;
plot(kalmanP_hat.time, kalmanP_hat.signals.values);

%% Comparing values
load('part3_estimator_x.mat');
load('part3_encoders_x.mat')
figure('Color', 'white', 'Name', 'Esimator output - pole gain 6')

grid

%pitch
subplot(5,1,1)
plot(kalmanX_hat.time, kalmanX_hat.signals.values(:,1),'b')
hold on
plot(simout.time, simout.signals.values(:,3),'r')
title("Pitch")
legend("Estimated", "Encoder")

%pitchrate
subplot(5,1,2)
plot(kalmanX_hat.time, kalmanX_hat.signals.values(:,2),'b')
hold on
plot(simout.time, simout.signals.values(:,4),'r')
title("Pitch-rate")

%elevation
subplot(5,1,3)
plot(kalmanX_hat.time, kalmanX_hat.signals.values(:,3),'b')
hold on
plot(simout.time, (simout.signals.values(:,5)-0.5),'r')
title("Elevation")

subplot(5,1,4)
plot(kalmanX_hat.time, kalmanX_hat.signals.values(:,4),'b')
hold on
plot(simout.time, simout.signals.values(:,6),'r')
title("Elevation-rate")

subplot(5,1,5)
plot(kalmanX_hat.time, kalmanX_hat.signals.values(:,5),'b')
hold on
plot(simout.time, simout.signals.values(:,2),'r')
title("Travel-rate")