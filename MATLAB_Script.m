clear; clc; close all;
% Cruise Control System - MATLAB Script

%% ================== PARAMETERS ==================
% Define parameters of the actuator (DC motor) and vehicle dynamics

J = 1.1 * 100;   % Moment of inertia of the motor + wheels [kg·m^2]
b = 0.00000224;  % Viscous damping coefficient [N·m/(rad/s)]
L = 1e-6;        % Armature inductance [H]
K = 0.12;        % Motor constant [rad/s/V] (Back EMF & Torque constant)
R = 0.04;        % Armature resistance [Ω]

% Tachometer parameters (used for measuring angular velocity)
Ktwo = 24.8;  % Tachometer gain [V/(rad/s)]
Rtwo = 40;    % Tachometer resistance [Ω]
Ltwo = 0.024; % Tachometer inductance [H]

%% ================== TRANSFER FUNCTIONS ==================
% Define the PID controller transfer function
C = pid(30, 1.0, 0); % PID gains: Kp = 30, Ki = 1.0, Kd = 0

% Define the plant transfer function (motor dynamics)
s = tf('s'); % Define Laplace variable
G = tf(K, [(J * L), (J * R + b * L), (K * K)]);

% Define the tachometer (sensor) transfer function
H = tf(Ktwo * Rtwo, [(Ltwo), (Rtwo)]);

% Define the sensitivity function (how the system reacts to disturbances)
S = 1 / (1 + C * G);

% Define the closed-loop transfer function
sys = feedback(C * G, H / Ktwo);

%% ================== ROOT LOCUS ANALYSIS ==================
% Solve for Kp as the varied parameter using L(s) = s*G*H/(Ki*G*H + s)
% Normalize H to have unity gain at steady state
H_norm = H / Ktwo;
L = tf([1 0], 1) * G * H_norm / (C.Ki * G * H_norm + tf([1 0], 1));

figure;
rlocus(L);
title('Root Locus Plot');
fontsize(scale=1.25);

%% ================== BODE PLOTS ==================
% Frequency response analysis

% Closed-loop Bode plot
figure;
bode(sys);
title('Closed-Loop Bode Plot');

% Disturbance rejection Bode plot
figure;
bode(S);
title('Disturbance Rejection Bode Plot');

% Gain and phase margins
figure;
margin(C * G);
title('Gain and Phase Margins');

%% ================== STEP RESPONSE ==================
% Uncomment to visualize the step response
% figure;
% step(sys);
% title('Step Response');

% Get step response characteristics
stepInformation = stepinfo(sys);

%% ================== SIMULINK RESPONSE ==================
% Prepare parameters for Simulink model

% PID controller parameters
[numC, denomC] = tfdata(C);
numC = cell2mat(numC);
denomC = cell2mat(denomC);
Kp = C.Kp;
Ki = C.Ki;
Kd = C.Kd;

% Plant parameters
[numG, denomG] = tfdata(G);
numG = cell2mat(numG);
denomG = cell2mat(denomG);

% Sensor (tachometer) parameters
[numH, denomH] = tfdata(H);
numH = cell2mat(numH);
denomH = cell2mat(denomH);

% Disturbance parameters for Simulink
disturbanceSign = -1; % -1 means disturbance decreases speed, +1 increases speed
distTime = 7;         % Time at which step disturbance is applied [s]
stepGain = 1;         % Magnitude of step input disturbance

% Run Simulink model ('controlSys.slx') for 15 seconds
simout = sim('controlSys.slx', 'StopTime', "15");
tout = simout.tout;

%% ================== PROCESS SIMULATION OUTPUT ==================
% Allow shifting the initial time if needed
startInd = 1; % 1 => Simulink output unchanged
tout = tout(startInd:end) - tout(startInd);
yout = simout.yout{1}.Values.Data(startInd:end);

% Plot the simulation results
figure;
plot(tout, yout, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Simulink Response - Angular Velocity vs. Time');
grid on;
