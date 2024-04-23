clc;
clear all;
close all;
% Define the transfer function
numerator = [1 15]; % numerator coefficients
denominator= [1, 3 ,0] ; % denominator coefficients
G = tf(numerator, denominator);
% Generate Bode plots
figure;
bode (G) ;
% Find gain and phase margins
[Gm, Pm, Wcg, Wcp] = margin(G);
disp(['Gain margin:', num2str(Gm)]);

disp(['Phase margin: ', num2str(Pm)]);
disp(['Crossover frequency for gain margin:', num2str(Wcg)]);
disp(['Crossover frequency for phase margin:', num2str(Wcp)]);

