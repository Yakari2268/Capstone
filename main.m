clc; clear; close all;

%% -------------------------------------------------
% PARAMETERS
%% -------------------------------------------------
fs = 5000;                 % Sampling frequency
t = 0:1/fs:1;              % Time vector (1 second)
f = 50;                    % Power system frequency

% Lorenz system parameters (chaotic)
a = 10;
b = 28;
c = 8/3;

dt = 1e-4;                 % Integration step
N = length(t);

%% -------------------------------------------------
% POWER QUALITY SIGNAL GENERATION
%% -------------------------------------------------

% Normal signal
normal = sin(2*pi*f*t);

% Voltage sag
sag = 0.5 * sin(2*pi*f*t);

% Voltage swell
swell = 1.5 * sin(2*pi*f*t);

% Interruption
interruption = zeros(size(t));
interruption(1:round(N/3)) = sin(2*pi*f*t(1:round(N/3)));

% Harmonics (5th harmonic added)
harmonics = sin(2*pi*f*t) + 0.2*sin(2*pi*5*f*t);

% Choose signal to test
input_signal = sag;    % <-- change here (normal, sag, swell, etc.)

% Add noise (5%)
noise = 0.05 * randn(size(t));
measured_signal = input_signal + noise;

%% -------------------------------------------------
% LORENZ CHAOTIC SYSTEM (MASTER & SLAVE)
%% -------------------------------------------------

% Initial conditions
x = [1; 1; 1];   % Master
y = [2; 2; 2];   % Slave

% Storage
ex = zeros(1,N);
ey = zeros(1,N);

for k = 1:N
    % Master system
    dx1 = a*(x(2) - x(1));
    dx2 = b*x(1) - x(1)*x(3) - x(2);
    dx3 = x(1)*x(2) - c*x(3);

    x = x + dt*[dx1; dx2; dx3];

    % Slave system (signal injected)
    dy1 = a*(y(2) - y(1)) + measured_signal(k);
    dy2 = b*y(1) - y(1)*y(3) - y(2);
    dy3 = y(1)*y(2) - c*y(3);

    y = y + dt*[dy1; dy2; dy3];

    % Error dynamics
    e = y - x;

    ex(k) = e(1);
    ey(k) = e(2);
end

%% -------------------------------------------------
% CHAOTIC SCATTER PLOT (Paper Fig. 8–15)
%% -------------------------------------------------

figure;
scatter(ex, ey, 2, 'filled');
xlabel('e_1');
ylabel('e_2');
title('Chaotic Scatter (Lorenz Error Dynamics)');
grid on;

%% -------------------------------------------------
% FEATURE EXTRACTION (Paper Eq. 11)
%% -------------------------------------------------

idx1 = find(ex > -1.5 & ex <= -0.5 & ey > 0);
idx2 = find(ex > -0.5 & ex <=  0.5 & ey > 0);
idx3 = find(ex >  0.5 & ex <=  1.5 & ey > 0);

c1 = mean(abs(ey(idx1)));
c2 = mean(abs(ey(idx2)));
c3 = mean(abs(ey(idx3)));

features = [c1 c2 c3];

disp('Extracted Chaotic Features [c1 c2 c3] =');
disp(features);
