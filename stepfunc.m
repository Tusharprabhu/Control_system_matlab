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