clc;
close all;
clear all;

num = [1 15];
den = [1 3 0];
G = tf(num, den);

% DC gain for each input type
kp = dcgain(G);
ess1 = 1 / (1 + kp); % Steady-state error for step input

G1 = tf(conv(num, [1 0]), den);
kv = dcgain(G1);
ess2 = 1 / kv; % Steady-state error for ramp input

G2 = tf(conv(num, [1 0 0]), den);
ka = dcgain(G2);
ess3 = 1 / ka; % Steady-state error for parabolic input

% Plot step response
t = 0:0.01:10;
[y_step, t_step] = step(G, t);
figure;
plot(t_step, y_step);
xlabel('Time');
ylabel('Output');
title('Step Response');
grid on;

disp(['Steady-State Error for Step Input: ', num2str(ess1)]);
disp(['Steady-State Error for Ramp Input: ', num2str(ess2)]);
disp(['Steady-State Error for Parabolic Input: ', num2str(ess3)]);