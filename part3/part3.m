load ../part2/part2
addpath(data)

% IMU
PORT = 4;

%% IMU plot
close all

load('part3_1_comparison')
figure('Color', 'white', 'Name', 'Gyroscope VS Encoders')

plot(comparison.time, comparison.signals.values(:,1))
hold on
plot(comparison.time, comparison.signals.values(:,2))
plot(comparison.time, comparison.signals.values(:,3))
plot(comparison.time, comparison.signals.values(:,4))
plot(comparison.time, comparison.signals.values(:,5))
plot(comparison.time, comparison.signals.values(:,6))

grid
title('Gyroscope VS Encoders (offset)')
xlabel('t/[s]')
ylabel('\theta/[rad]')
legend('dot p (IMU)', 'dot e (IMU)', 'dot lambda (IMU)', 'dot lambda', 'dot p', 'dot e', 'Location', 'southeast')


%%
close all
load('part3_1_comparison_off')
figure('Color', 'white', 'Name', 'Gyroscope VS Encoders  (offset)')


plot(comparison.time, comparison.signals.values(:,4))
hold on
plot(comparison.time, comparison.signals.values(:,5))
plot(comparison.time, comparison.signals.values(:,6))
plot(comparison.time, comparison.signals.values(:,1))
plot(comparison.time, comparison.signals.values(:,2))
plot(comparison.time, comparison.signals.values(:,3))

grid
title('Gyroscope VS Encoders')
xlabel('t/[s]')
ylabel('\theta/[rad]')
legend('dot lambda', 'dot p', 'dot e','dot p (IMU)', 'dot e (IMU)', 'dot lambda (IMU)', 'Location', 'southeast')

%%
load('part3_1_accelerometer')
figure('Color', 'white', 'Name', 'Accelerometer')

plot(accelerometer.time, accelerometer.signals.values(:,1))
hold on
plot(accelerometer.time, accelerometer.signals.values(:,2))
plot(accelerometer.time, accelerometer.signals.values(:,3))

grid
title('Accelerometer')
xlabel('t/[s]')
ylabel('m/s^2')
legend('ax', 'ay', 'az',  'Location', 'southeast')

load('part3_1_anglesFromAcc')
load('part3_1_anglesFromEnc')
figure('Color', 'white', 'Name', 'Accelerometer angles vs Encoder angles')

plot(anglesFromAcc.time, anglesFromAcc.signals.values(:,1))
hold on
plot(anglesFromAcc.time, anglesFromAcc.signals.values(:,2))
plot(anglesFromEnc.time, anglesFromEnc.signals.values(:,1))
plot(anglesFromEnc.time, anglesFromEnc.signals.values(:,2))

grid
title('Accelerometer angles vs Encoder angles')
xlabel('t/[s]')
ylabel('\theta')
legend('p (Acc)', 'e (Acc)', 'p (Enc)', 'e (Enc)', 'Location', 'southeast')
%%
load('part3_2_eulerRate')
load('part3_2_angleRateFromEnc')
figure('Color', 'white', 'Name', 'IMU angle rate vs Encoder angle rate')

plot(eulerRates.time(3501:7501), eulerRates.signals.values(3501:7501,1))
hold on
plot(eulerRates.time(3501:7501), eulerRates.signals.values(3501:7501,2))
plot(eulerRates.time(3501:7501), eulerRates.signals.values(3501:7501,3))
plot(angleRateFromEnc.time(3501:7501), angleRateFromEnc.signals.values(3501:7501,1))
plot(angleRateFromEnc.time(3501:7501), angleRateFromEnc.signals.values(3501:7501,2))
plot(angleRateFromEnc.time(3501:7501), angleRateFromEnc.signals.values(3501:7501,3))

grid
title('IMU angle rate vs Encoder angle rate')
xlabel('t/[s]')
ylabel('\theta')
legend('dot p (IMU)', 'dit e (IMU)', 'dot lambda (IMU)', 'dot p (Enc)', 'dot e (Enc)', 'dot lambda (Enc)', 'Location', 'southeast')

%% Estimator
clc

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

A = [0 1 0 0 0; 0 0 0 0 0; 0 0 0 1 0; 0 0 0 0 0; K_3 0 0 0 0];
B = [0 0; 0 K_1; 0 0; K_2 0; 0 0];
C = eye(5);

poles = eig(A01-B01*K);
fastPole = max(abs(poles));

poleGain = 20;
poleLength = fastPole*poleGain;
maxAngle = pi/8;
angles = -maxAngle : maxAngle/2 : maxAngle;

newPoles = -poleLength*exp(i*angles)

p3 = 50*[-0.3, -0.26-0.15*i, -0.26+0.15*i, -0.15-0.26*i, -0.15+0.26*i];

L3 = place(A', C', p3)';

L=place(A',C',newPoles);

%% Minimal states e and travelrate

C = diag([1 0 1 1 1]);

poles = eig(A01-B01*K);
fastPole = max(abs(poles));
poleGain = 1;

poleLength = fastPole*poleGain;
maxAngle = pi/8;
angles = -maxAngle : maxAngle/2 : maxAngle;
newPoles = -poleLength*exp(i*angles);

L=place(A',C',newPoles);

%%
close all
figure()
hold on
grid
xlim([-45 7])
xL = xlim;
ylim([-20 20])
yL = ylim;
line([0 0], yL,'Color', 'black');
line(xL, [0 0],'Color', 'black');
title('Poleplacement')
xlabel('Re')
ylabel('Im')

for k = 1:5
    plot(real(newPoles(k)), imag(newPoles(k)), 'r*')   
end

%%
figure('Color', 'white', 'Name', 'Accelerometer angles vs Encoder angles')

plot(anglesFromAcc.time, anglesFromAcc.signals.values(:,1))
hold on
plot(anglesFromAcc.time, anglesFromAcc.signals.values(:,2))
plot(anglesFromEnc.time, anglesFromEnc.signals.values(:,1))
plot(anglesFromEnc.time, anglesFromEnc.signals.values(:,2))

grid
title('Accelerometer angles vs Encoder angles')
xlabel('t/[s]')
ylabel('\theta')
legend('p (Acc)', 'e (Acc)', 'p (Enc)', 'e (Enc)', 'Location', 'southeast')

%% Plotting poles
close all
figure()
hold on
grid
xlim([-7 7])
xL = xlim;
ylim([-7 7])
yL = ylim;
line([0 0], yL,'Color', 'black');
line(xL, [0 0],'Color', 'black');

for k = 1:5
    plot(real(p1(k)), imag(p1(k)), 'r*')   
end

figure()
hold on
grid
xlim([-2 2])
xL = xlim;
ylim([-2 2])
yL = ylim;
line([0 0], yL,'Color', 'black');
line(xL, [0 0],'Color', 'black');

for k = 1:5
    plot(real(p2(i)), imag(p2(i)), 'r*')   
end

figure()
hold on
grid
xlim([-7 7])
xL = xlim;
ylim([-7 7])
yL = ylim;
line([0 0], yL,'Color', 'black');
line(xL, [0 0],'Color', 'black');

for k = 1:5
    plot(real(p3(i)), imag(p3(i)), 'r*')   
end

%% Comparing values
load('part3_estimator_x.mat');
load('part3_encoders_x.mat');
figure('Color', 'white', 'Name', 'Esimator output - pole gain 6')

grid
%pitch
subplot(5,1,1)
plot(estimatedX.time, estimatedX.signals.values(:,1),'b')
hold on
plot(simout.time, simout.signals.values(:,3),'r')
title("Pitch")
legend("Estimated", "Encoder")
%pitchrate
subplot(5,1,2)
plot(estimatedX.time, estimatedX.signals.values(:,2),'b')
hold on
plot(simout.time, simout.signals.values(:,4),'r')
title("Pitch-rate")

%elevation
subplot(5,1,3)
plot(estimatedX.time, estimatedX.signals.values(:,3),'b')
hold on
plot(simout.time, (simout.signals.values(:,5)-0.5),'r')
title("Elevation")

subplot(5,1,4)
plot(estimatedX.time, estimatedX.signals.values(:,4),'b')
hold on
plot(simout.time, simout.signals.values(:,6),'r')
title("Elevation-rate")

subplot(5,1,5)
plot(estimatedX.time, estimatedX.signals.values(:,5),'b')
hold on
plot(simout.time, simout.signals.values(:,2),'r')
title("Travel-rate")

%%
close all

load('part3_unstable_p_polegain_50.mat');
figure('Color', 'white', 'Name', 'Esimator output - pole gain 6')

hold on
grid

plot(estimatedX.time, estimatedX.signals.values(:,1))
plot(estimatedX.time, estimatedX.signals.values(:,2))
plot(estimatedX.time, estimatedX.signals.values(:,3))
plot(estimatedX.time, estimatedX.signals.values(:,4))
plot(estimatedX.time, estimatedX.signals.values(:,5))

title('Esimator output - Pole gain 6')
xlabel('t/[s]')
ylabel('q')
legend('p', 'p_{dot} ', 'e', 'e_{dot}', 'lambda', 'Location', 'southeast')

load('part3_stable_p_realPolegain_10.mat');
figure('Color', 'white', 'Name', 'Esimator output - pole gain 10')

hold on
grid

plot(estimatedX.time, estimatedX.signals.values(:,1))
plot(estimatedX.time, estimatedX.signals.values(:,2))
plot(estimatedX.time, estimatedX.signals.values(:,3))
plot(estimatedX.time, estimatedX.signals.values(:,4))
plot(estimatedX.time, estimatedX.signals.values(:,5))

title('Esimator output - Pole gain 10')
xlabel('t/[s]')
ylabel('q')
legend('p', 'p_{dot} ', 'e', 'e_{dot}', 'lambda', 'Location', 'southeast')

load('part3_stable_p_realPolegain_14.mat');
figure('Color', 'white', 'Name', 'Esimator output - pole gain 14')

hold on
grid

plot(estimatedX.time, estimatedX.signals.values(:,1))
plot(estimatedX.time, estimatedX.signals.values(:,2))
plot(estimatedX.time, estimatedX.signals.values(:,3))
plot(estimatedX.time, estimatedX.signals.values(:,4))
plot(estimatedX.time, estimatedX.signals.values(:,5))

title('Esimator output - Pole gain 14')
xlabel('t/[s]')
ylabel('q')
legend('p', 'p_{dot} ', 'e', 'e_{dot}', 'lambda', 'Location', 'southeast')

load('part3_stable_p_realPolegain_16.mat');
figure('Color', 'white', 'Name', 'Esimator output - pole gain 16')

hold on
grid

plot(estimatedX.time, estimatedX.signals.values(:,1))
plot(estimatedX.time, estimatedX.signals.values(:,2))
plot(estimatedX.time, estimatedX.signals.values(:,3))
plot(estimatedX.time, estimatedX.signals.values(:,4))
plot(estimatedX.time, estimatedX.signals.values(:,5))

title('Esimator output - Pole gain 16')
xlabel('t/[s]')
ylabel('q')
legend('p', 'p_{dot} ', 'e', 'e_{dot}', 'lambda', 'Location', 'southeast')

load('part3_stable_p_realPolegain_14_adjusted_Q_p_100.mat');
figure('Color', 'white', 'Name', 'Esimator output - pole gain 14 (Tweaked Q)')

hold on
grid

plot(estimatedX.time, estimatedX.signals.values(:,1))
plot(estimatedX.time, estimatedX.signals.values(:,2))
plot(estimatedX.time, estimatedX.signals.values(:,3))
plot(estimatedX.time, estimatedX.signals.values(:,4))
plot(estimatedX.time, estimatedX.signals.values(:,5))

title('Esimator output - Pole gain 14 (Tweaked Q)')
xlabel('t/[s]')
ylabel('q')
legend('p', 'p_{dot} ', 'e', 'e_{dot}', 'lambda', 'Location', 'southeast')