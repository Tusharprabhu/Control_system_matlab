% Define the transfer function
num = [1 15];
den = [1 3 0];

% Create the transfer function object
G = tf(num, den);

% Compute the roots of the characteristic equation
poles = roots(den);

% Check if the poles are in the left-half plane
stable = all(real(poles) < 0);

if stable
    disp('The system is stable.')
else
    disp('The system is unstable.')
end

% Plot the root locus
figure;
rlocus(G);
title('Root Locus Plot');
xlabel('Real Axis');
ylabel('Imaginary Axis');