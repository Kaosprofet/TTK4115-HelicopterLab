close all
clear
clc

Part1Task2

load('test1_SimControl.mat')
figure('Color', 'white', 'Name', 'Step responce to equilibrium point')

plot(simout.time, simout.signals.values(:,1))
hold on
plot(simout.time, simout.signals.values(:,4))
plot(simout.time, simout.signals.values(:,5)-0.5)

grid
title('Step response from rest to equilibrium')
xlabel('t/[s]')
ylabel('\theta/[rad]')
legend('\lambda - Travel','p - Pitch','e - Elevation', 'Location', 'southeast')

load('test2_SimWithTravelControl.mat')
figure('Color', 'white', 'Name', 'Step responce to equilibrium point w/travel controll')

plot(simout.time, simout.signals.values(:,1))
hold on
plot(simout.time, simout.signals.values(:,3))
plot(simout.time, simout.signals.values(:,5)-0.5)

grid
title('Step response from rest to equilibrium w/travel controll')
xlabel('t/[s]')
ylabel('\theta/[rad]')
legend('\lambda - Travel','p - Pitch','e - Elevation', 'Location', 'southeast')