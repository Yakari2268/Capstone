function lorenz_sync
    % Clear workspace and figures
    clear; clc; close all;

    % ---------------------------------------------------------
    % 1. System Parameters 
    % ---------------------------------------------------------
    sigma = 20;      % Modified Prandtl number for this paper
    r     = 28;      % Rayleigh number
    b     = 8/3;     % Geometric factor
    
    % Control Gains [cite: 303, 446]
    % k11 corresponds to u1 (set to 0 in paper's proof)
    % k22 corresponds to u2
    % k33 corresponds to u3
    k11 = 0;         
    k22 = -1;        
    k33 = -8/3;      

    % ---------------------------------------------------------
    % 2. Initial Conditions [cite: 446, 447]
    % ---------------------------------------------------------
    % Drive System (x1, x2, x3)
    x0 = [10; -10; 20];
    
    % Response System (y1, y2, y3)
    % Using the 'chaotic' start values to demonstrate synchronization
    y0 = [7; -10; 10]; 
    
    initial_state = [x0; y0];

    % ---------------------------------------------------------
    % 3. Time Integration
    % ---------------------------------------------------------
    t_span = [0 10]; % Simulation time 0 to 10s [cite: 421]
    
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
    [t, states] = ode45(@(t, S) system_dynamics(t, S, sigma, r, b, k11, k22, k33), ...
                        t_span, initial_state, options);

    % Extract states
    x1 = states(:, 1); x2 = states(:, 2); x3 = states(:, 3);
    y1 = states(:, 4); y2 = states(:, 5); y3 = states(:, 6);

    % Calculate Synchronization Errors (e = y - x)
    e1 = y1 - x1;
    e2 = y2 - x2;
    e3 = y3 - x3;

    % ---------------------------------------------------------
    % 4. Visualization
    % ---------------------------------------------------------
    figure('Name', 'Lorenz Chaos Synchronization', 'Color', 'w', 'Position', [100 100 1000 600]);

    % Subplot 1: Trajectories in Phase Space
    subplot(2, 2, [1 3]);
    plot3(x1, x2, x3, 'b', 'LineWidth', 0.5); hold on;
    plot3(y1, y2, y3, 'r--', 'LineWidth', 1);
    xlabel('x_1, y_1'); ylabel('x_2, y_2'); zlabel('x_3, y_3');
    legend('Drive System (x)', 'Response System (y)', 'Location', 'best');
    title('Phase Space Synchronization');
    grid on; view(45, 30);

    % Subplot 2: Time Series Comparison of State 1
    subplot(2, 2, 2);
    plot(t, x1, 'b'); hold on;
    plot(t, y1, 'r--');
    ylabel('State x_1 / y_1');
    legend('Drive', 'Response');
    title('State x_1 vs y_1');
    grid on;

    % Subplot 3: Synchronization Error Dynamics [cite: 420]
    subplot(2, 2, 4);
    plot(t, e1, 'k', 'LineWidth', 1); hold on;
    plot(t, e2, 'g', 'LineWidth', 1);
    plot(t, e3, 'm', 'LineWidth', 1);
    xlabel('Time (t)'); ylabel('Error Amplitude');
    legend('e_1 (y_1-x_1)', 'e_2 (y_2-x_2)', 'e_3 (y_3-x_3)');
    title('Error Dynamics (Convergence to 0)');
    grid on;
end

% ---------------------------------------------------------
% Helper Function: Coupled System Dynamics
% ---------------------------------------------------------
function dS = system_dynamics(~, S, sigma, r, b, k11, k22, k33)
    % Unpack state vector
    x1 = S(1); x2 = S(2); x3 = S(3);
    y1 = S(4); y2 = S(5); y3 = S(6);

    % Calculate Errors [cite: 298]
    e1 = y1 - x1;
    e2 = y2 - x2;
    e3 = y3 - x3;

    % Calculate Control Inputs [cite: 295, 303]
    u1 = k11 * e1;
    u2 = k22 * e2;
    u3 = k33 * e3;

    % Drive System Equations (Standard Lorenz) [cite: 280, 286]
    dx1 = sigma * (x2 - x1);
    dx2 = r * x1 - x2 - x1 * x3;
    dx3 = -b * x3 + x1 * x2;

    % Response System Equations (Lorenz + Control) [cite: 291-293]
    dy1 = sigma * (y2 - y1) + u1;
    dy2 = r * y1 - y2 - y1 * y3 + u2;
    dy3 = -b * y3 + y1 * y2 + u3;

    % Return derivatives
    dS = [dx1; dx2; dx3; dy1; dy2; dy3];
end