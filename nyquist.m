clc;
clear all;
close all;
% Define the transfer function
G=tf([1 15], [1 3 0]);
H=tf([1],[1]);


K=1;
GH= series(G, H);
sys = feedback(K * GH, 1);
% Generate Nyquist plot
nyquistplot(sys);
% Calculate gain and phase margins
[Gm, Pm, Wcg, Wcp]= margin(sys);
disp(['Gain margin: ',num2str(Gm) ] ) ;
disp(['Phase margin: ',num2str(Pm) ] ) ;
