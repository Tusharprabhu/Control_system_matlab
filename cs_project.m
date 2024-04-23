%Pole zeros
% Define the transfer function coefficients
num = [1 15];
den = [1 3 0];

% Create the transfer function object
sys = tf(num, den);

% Calculate poles and zeros
p = pole(sys);
z = zero(sys);

% Display the poles and zeros
disp('Poles:');
disp(p);
disp('Zeros:');
disp(z);

%%
%steady state error calculations
clc;close all;clear all;
num = [1 15];
den = [1 3 0];
G=tf(num,den)
kp=dcgain(G)
ess1=1/(kp+1)

G1=tf(conv(num,[1 0]),den)
kv=dcgain(G1)
ess2=1/kv

G2=tf(conv(num,[1 0 0]),den)
ka=dcgain(G2)
ess3=1/ka
%%
%pole zero plot
s=tf('s')
g1=(s+15)/(s^2+3*s+0)
pzmap(g1)
%%
%step input
%response for unit step input
% Define the transfer function of the plant
num = [1 15];
den = [1 3 0]; % Denominator coefficients
G = tf(num, den); % Transfer function representation

% Simulate the response to a unit step input
t = 0:0.01:10; % Time vector
u = ones(size(t)); % Unit step input
[y, t] = step(G, t); % Simulate the response

% Plot the response
plot(t, y, 'b', 'LineWidth', 2);
xlabel('Time');
ylabel('Output');
title('Response of the Plant to Unit Step Input');
grid on;
%%
%root locus
num = [1 15];
den = [1 3 0]; % Denominator coefficients
G = tf(num, den); % Transfer function representation
figure;
rlocus(G); % Plot the root locus
%%
% bode plot
num = [1 15];
den = [1 3 0]; % Denominator coefficients
G = tf(num, den); % Transfer function representation
[GM, PM, wgm, wpm] = margin(G);
bode(G); % Plot Bode plots
fprintf('Gain Margin (dB): %f\n', 20*log10(GM));
fprintf('Phase Margin (degrees): %f\n', PM);

% Plot Bode plots
bode(G);
grid on;
%%
%nyquist plot
num = [1 15];
den = [1 3 0]; % Denominator coefficients
G = tf(num, den); % Transfer function representation

% Compute gain and phase margins
[GM, PM, ~, ~] = margin(G);

fprintf('Gain Margin (dB): %f\n', 20*log10(GM));
fprintf('Phase Margin (degrees): %f\n', PM);

% Plot Nyquist plot using MATLAB's nyquist command
figure;
nyquist(G);
title('Nyquist Plot (MATLAB)');
%%
clc;
clear;
close all;
warning('off','Control:analysis:stepinfo2');

G = tf([1 15], [1 3 0]);

% Using Ziegler- Nichols method to tune controllers

% Choosing Ku to reach marginal stability
Ku = 0.01;
for k=2.8999:0.0001:3
    T = feedback(G*k, 1);
    S = stepinfo(T);
    if S.PeakTime == inf
        break;
    else
        Ku = k;
    end
end

GC = Ku;
T = feedback(G*GC, 1);
fprintf("\n\nTF of Marginally stable system");
T
figure;
step(T);
title('Step response of system with controller gain Ku');

% Measuring time period from graph, we get
fprintf("Ultimate gain Ku = %0.5f\n", Ku);
Tu = 0.542 - 0.181;
fprintf("Ultimate time period Tu = %0.5f\n", Tu);


% Next, we design controllers based on Ziegler-Nichols tuning rules

% P controller

GC = 0.5 * Ku;
T = feedback(G*GC, 1);

fprintf("\n\nTF of system with P Controller");
T
info = stepinfo(T);
fprintf("Settling Time = %0.6fs\n", info.SettlingTime);
fprintf("Overshooot percentage: %0.6f%%\n", info.Overshoot);

figure;
step(T);
title("Step response of P Controller");


% PI Controller
Kp =  0.45 * Ku;
Ki = 0.54 * Ku / Tu;
GC = tf(Kp, 1)  + tf(Ki, [1 0]);
T = feedback(G*GC, 1);

fprintf("\n\nTF of system with PI Controller");
T
info = stepinfo(T);
fprintf("Settling Time = %0.6fs\n", info.SettlingTime);
fprintf("Overshooot percentage: %0.6f%%\n", info.Overshoot);

figure;
step(T);
title("Step response of PI Controller");


% PD Controller
Kp =  0.8 * Ku;
Kd = 0.1 * Ku * Tu;
GC = tf(Kp, 1)  + tf([Kd 0], 1);
T = feedback(G*GC, 1);

fprintf("\n\nTF of system with PD Controller");
T
info = stepinfo(T);
fprintf("Settling Time = %0.6fs\n", info.SettlingTime);
fprintf("Overshooot percentage: %0.6f%%\n", info.Overshoot);

figure;
step(T);
title("Step response of PD Controller");

% PID Controller
Kp =  0.6 * Ku;
Ki = 1.2 * Ku / Tu;
Kd = 0.075 * Ku * Tu;
GC = tf(Kp, 1) + tf(Ki, [1 0]) + tf([Kd 0], 1);
T = feedback(G*GC, 1);

fprintf("\n\nTF of system with PID Controller");
T
info = stepinfo(T);
fprintf("Settling Time = %0.6fs\n", info.SettlingTime);
fprintf("Overshooot percentage: %0.6f%%\n", info.Overshoot);

figure;
step(T);
title("Step response of PID Controller");


% We see that the Ziegler Nichols tuning is aggressive and optimizes for
% fast settling time at the expense of high overshoot.
% So, we modify the tuning to allow for our condition for lower peak
% overshoot to also be met.

% Modified PID Controller
Kp =  0.2* Ku ;
Ki = 1.2 * Ku / Tu;
Kd = 0.075 * Ku * Tu;

GC = tf(Kp, 1) + tf(Ki, [1 0]) + tf([Kd 0], 1);
T = feedback(G*GC, 1);

fprintf("\n\nTF of system with modified PID Controller");
T
info = stepinfo(T);
fprintf("Settling Time = %0.6fs\n", info.SettlingTime);
fprintf("Overshooot percentage: %0.6f%%\n", info.Overshoot);

figure;
step(T);
title("Step response of PID Controller");


%%
% Plant transfer function
num_plant = [1 15];
den_plant = [1 3 0];
G = tf(num_plant, den_plant);

% Desired performance specifications
peak_overshoot = 10;   % Less than 10%
settling_time = 2;     % Less than 2 seconds

% Design controllers
disp('Designing controllers...');

% (a) Proportional (P) controller
Kp = 1;     % Initial guess for Kp
C_P = Kp;   % C(s) = Kp

% (b) Proportional + Integral (PI) controller
Kp = 2;     % Initial guess for Kp
Ki = 1;     % Initial guess for Ki
C_PI = tf([Kp Ki], [1 0]);   % C(s) = Kp + Ki/s

% (c) Proportional + Derivative (PD) controller
Kp = 2;     % Initial guess for Kp
Kd = 1;     % Initial guess for Kd
C_PD = tf([Kp Kp*Kd], [1 0]);   % C(s) = Kp + Kd*s

% (d) Proportional + Integral + Derivative (PID) controller
Kp = 3;     % Initial guess for Kp
Ki = 1;     % Initial guess for Ki
Kd = 1;     % Initial guess for Kd
C_PID = tf([Kp Kp*Kd Kp*Ki], [1 0 0]);   % C(s) = Kp + Kd*s + Ki/s

% Simulate and evaluate performance
disp('Simulating closed-loop responses...');

% (a) P controller
T_P = feedback(C_P*G, 1);
[y_P, t_P] = step(T_P);
[peakValue_P, peakTime_P] = peak(y_P);
settling_time_P = settling_time(y_P, t_P, 0.02);
disp('P controller performance:');
disp(['Peak overshoot = ' num2str(100*(peakValue_P-1)) '%']);
disp(['Settling time = ' num2str(settling_time_P) ' seconds']);

% (b) PI controller
T_PI = feedback(C_PI*G, 1);
[y_PI, t_PI] = step(T_PI);
[peakValue_PI, peakTime_PI] = peak(y_PI);
settling_time_PI = settling_time(y_PI, t_PI, 0.02);
disp('PI controller performance:');
disp(['Peak overshoot = ' num2str(100*(peakValue_PI-1)) '%']);
disp(['Settling time = ' num2str(settling_time_PI) ' seconds']);

% (c) PD controller
T_PD = feedback(C_PD*G, 1);
[y_PD, t_PD] = step(T_PD);
[peakValue_PD, peakTime_PD] = peak(y_PD);
settling_time_PD = settling_time(y_PD, t_PD, 0.02);
disp('PD controller performance:');
disp(['Peak overshoot = ' num2str(100*(peakValue_PD-1)) '%']);
disp(['Settling time = ' num2str(settling_time_PD) ' seconds']);

% (d) PID controller
T_PID = feedback(C_PID*G, 1);
[y_PID, t_PID] = step(T_PID);
[peakValue_PID, peakTime_PID] = peak(y_PID);
settling_time_PID = settling_time(y_PID, t_PID, 0.02);
disp('PID controller performance:');
disp(['Peak overshoot = ' num2str(100*(peakValue_PID-1)) '%']);
disp(['Settling time = ' num2str(settling_time_PID) ' seconds']);
%%
num=[1 15] ;
den=[1 3 0];
sys=tf(num, den);
bode(sys);
nyquistplot(sys);
Itiview('nyquist', sys);
% grid on;
% xlabel( ' real ) ;
% ylabel( imaginary'
% title( 'Nyquist Plot' ) ;
[gm,pm, wgcf,wpcf]=margin(sys)
%%
numerator = [1 15];
denominator = [1 3 0];
plant_sys = tf(numerator, denominator);
% Desired performance criteria
desired_overshoot = 0.10; % 10%
desired_settling_time = 2; % seconds
% Proportional (P) Controller
Kp_P = 10; % Adjust as needed
controller_P = tf(Kp_P, 1);
% Proportional-Integral (PI) Controller
Kp_PI = 10; % Adjust as needed
Ki_PI = 2; % Adjust as needed
controller_PI = tf([Kp_PI, Ki_PI], [1, 0]);
% Proportional-Derivative (PD) Controller
Kp_PD = 10; % Adjust as needed
Kd_PD = 2; % Adjust as needed
controller_PD = tf([Kd_PD, Kp_PD], 1);
% Proportional-Integral-Derivative (PID) Controller
Kp_PID = 3; % Adjust as needed
Ki_PID = 8; % Adjust as needed
Kd_PID = 2; % Adjust as n8eded
controller_PID = tf([Kd_PID, Kp_PID, Ki_PID], 1);
% Closed-loop systems with different controllers
closed_loop_P = feedback(plant_sys * controller_P, 1);
closed_loop_PI = feedback(plant_sys * controller_PI, 1);
closed_loop_PD = feedback(plant_sys * controller_PD, 1);
closed_loop_PID = feedback(plant_sys * controller_PID, 1);
% Time vector for simulation
t = 0:0.01:10; % Adjust time vector as needed
% Step response analysis using stepinfo
stepinfo_P = stepinfo(closed_loop_P);
stepinfo_PI = stepinfo(closed_loop_PI);
stepinfo_PD = stepinfo(closed_loop_PD);
stepinfo_PID = stepinfo(closed_loop_PID);
% Display settling time
disp("Settling Time Information:");
disp("----------------------------");
disp("Proportional (P) Controller:");
disp(stepinfo_P.SettlingTime);
disp("----------------------------");
disp("Proportional-Integral (PI) Controller:");
disp(stepinfo_PI.SettlingTime);
disp("----------------------------");

disp("Proportional-Derivative (PD) Controller:");
disp(stepinfo_PD.SettlingTime);
disp("----------------------------");
disp("Proportional-Integral-Derivative (PID) Controller:");
disp(stepinfo_PID.SettlingTime);
% Plot step response
figure;
step(closed_loop_P, t);
title('Proportional (P) Controller');
figure;
step(closed_loop_PI, t);
title('Proportional-Integral (PI) Controller');
figure;
step(closed_loop_PD, t);
title('Proportional-Derivative (PD) Controller');
figure; 
step(closed_loop_PID, t);
title('Proportional-Integral-Derivative (PID) Controller');
%%
num =[1 15];
den =[1 3 0];
G = tf(num, den);
t=0:0.01:10;
u = ones(size(t)); % Step input
[y,t]=step(G, t);
plot(t, y);
xlabel('Time');
ylabel('Output');
title('Step Response');

info = stepinfo(G);
max_overshoot = info.Overshoot; % Maximum peak overshoot
peak_time = info.PeakTime; % Peak time
delay_time = info.SettlingTime - info.RiseTime; % Delay time
settling_time = info.SettlingTime; % Settling time
risetime = info.RiseTime; % Rise time
fprintf('Maximum Peak Overshoot: %.2f\n', max_overshoot);
fprintf('Peak Time: %.2f\n', peak_time);
fprintf('Delay Time: %.2f\n', delay_time);
fprintf('Settling Time: %.2f\n', settling_time);
fprintf('Rise Time: %.2f\n', risetime);
