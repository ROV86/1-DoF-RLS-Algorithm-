% Initialization
time_steps = 2000;  % Number of steps in the experiment/simulation
m = 5;                % Mass of the ROV in kg
mA = 1;               % Added mass in kg
D_k = 0.8;            % Initial estimate of the drag coefficient in Ns/m
D_k_history = zeros(1, time_steps); % To store D_k history for plotting
D = 1;                % True value of drag

% Increase initial error covariance to reduce overconfidence
P_k = 50;             % Increased initial error covariance for more uncertainty
P_k_history = zeros(1, time_steps); % To store P_k history
R = 0.3;              % Increased measurement noise covariance to add uncertainty
dt = 0.01;            % Time between steps
T = 1;                % Thruster Allocation Matrix (T) - Simplified for 1 DOF
C_k = 0;              % Placeholder for Coriolis-like effect

tau_input = 5;        % Example thruster inputs

% Simulate Measured Velocities
v_measured = zeros(1, time_steps); % Pre-allocated array for measured velocities
v_predicted_arr = zeros(1, time_steps); % Pre-allocated array for predicted velocities

% Simulate velocities
for k = 2:time_steps
    tau = T * tau_input; % Convert input to actual thrust
    a = (tau - D * v_measured(k - 1) - C_k * v_measured(k - 1)) / (m + mA);
    v_measured(k) = v_measured(k - 1) + a * dt + normrnd(0, sqrt(R)); % Adding measurement noise
end

% Update Drag Coefficient Using RLS Algorithm
for k = 2:time_steps
    v_predicted = (T * tau_input - D_k * v_measured(k - 1) - C_k * v_measured(k - 1)) / (m + mA) * dt;
    v_predicted_arr(k) = v_predicted; % Store predicted velocity
    
    K_k = P_k / (P_k + R);  % Kalman gain
    D_k = D_k + K_k * (v_measured(k) - v_measured(k - 1) - v_predicted * dt); % Update D_k
    D_k_history(k) = D_k; % Storing D_k history
    
    % Apply a higher lower bound for error covariance
    P_k = max((1 - K_k) * P_k, 0.03); % Ensure P_k doesn't reach zero
    P_k_history(k) = P_k; % Store P_k history
end

% Plotting Results
figure;
subplot(3, 1, 1);
plot(1:time_steps, D_k_history); % Ensure correct plot syntax
title('Estimated Drag Coefficient (D_k) over Time');
xlabel('Time Step');
ylabel('D_k');

subplot(3, 1, 2);
plot(1:time_steps, v_measured, 'b', 1:time_steps, v_predicted_arr, 'r--'); % Corrected plot syntax
legend('Measured Velocity', 'Predicted Velocity');
title('Measured vs. Predicted Velocities');
xlabel('Time Step');
ylabel('Velocity');

subplot(3, 1, 3);
plot(1:time_steps, P_k_history); % Corrected plot syntax
title('Error Covariance (P_k) over Time');
xlabel('Time Step');
ylabel('P_k');

% Display the final estimated drag coefficient
fprintf('Final estimated drag coefficient D_k: %f\n', D_k);
