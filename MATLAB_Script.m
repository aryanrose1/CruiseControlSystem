%% ========================================================================
%  Cruise Control System - MATLAB Script
%  This script sets up and analyzes a PID-controlled DC motor system that
%  models automotive cruise control. It defines the system parameters,
%  constructs transfer functions, performs classical control analyses, and
%  runs a corresponding Simulink model for validation.
%  ========================================================================

clear; clc; close all;

%% ============================ PARAMETERS ================================
% 1) Actuator (DC Motor) and Vehicle Dynamics
%    - J: total moment of inertia combining motor + wheels/vehicle.
%    - b: viscous damping representing overall friction opposing motion.
%    - L, R: electrical inductance and resistance of the motor’s armature.
%    - K: motor constant (torque constant = back-EMF constant).
J = 1.1 * 100;    % [kg·m^2] Moment of inertia
b = 0.00000224;   % [N·m/(rad/s)] Viscous damping coefficient
L = 1e-6;         % [H] Motor armature inductance
K = 0.12;         % [rad/s/V] Motor/back-EMF constant
R = 0.04;         % [Ω] Armature resistance

% 2) Tachometer (Sensor) Parameters
%    - Ktwo: sensor gain for converting angular velocity to voltage output.
%    - Rtwo, Ltwo: internal resistance and inductance of the tachometer.
Ktwo = 24.8;      % [V/(rad/s)] Tachometer gain
Rtwo = 40;        % [Ω] Tachometer resistance
Ltwo = 0.024;     % [H] Tachometer inductance

%% ========================== TRANSFER FUNCTIONS ==========================
% Define the Laplace variable
s = tf('s');

% 1) PID Controller
%    - Kp, Ki, Kd are the proportional, integral, and derivative gains.
%      Adjust these to tune the controller. 
C = pid(30, 1.0, 0);  % Gains: Kp=30, Ki=1.0, Kd=0

% 2) Plant (G):
%    - Represents the DC motor and vehicle inertia in Laplace domain.
%      Numerator = K, Denominator = (J*L)*s^2 + (J*R + b*L)*s + (K*K).
G = tf(K, [(J * L), (J * R + b * L), (K * K)]);

% 3) Tachometer (H):
%    - Models the sensor that outputs a voltage proportional to angular velocity.
%      Numerator = (Ktwo * Rtwo), Denominator = (Ltwo*s + Rtwo).
H = tf(Ktwo * Rtwo, [Ltwo, Rtwo]);

% 4) Sensitivity Function (S):
%    - S = 1 / (1 + C*G) shows how well the system rejects disturbances.
S = 1 / (1 + C * G);

% 5) Closed-Loop Transfer Function (sys):
%    - feedback(...) creates a closed-loop system with controller C*G and 
%      feedback path given by H/Ktwo (i.e., normalized sensor gain).
sys = feedback(C * G, H / Ktwo);

%% ========================= ROOT LOCUS ANALYSIS ==========================
% Normalization of H to unity DC gain (H_norm) and formation of open-loop
% transfer function L used for root-locus analysis.
H_norm = H / Ktwo;
L = tf([1 0], 1) * G * H_norm / (C.Ki * G * H_norm + tf([1 0], 1));

figure;
rlocus(L);
title('Root Locus Plot');
% (Optional) Format font size if desired:
% fontsize(scale=1.25); % Might require a custom function or toolbox.

%% =========================== BODE PLOTS ================================
% Frequency response analysis to check stability margins and disturbance
% rejection performance.

% 1) Bode plot of the closed-loop system (sys)
figure;
bode(sys);
title('Closed-Loop Bode Plot');

% 2) Bode plot of the sensitivity function (S) for disturbance rejection
figure;
bode(S);
title('Disturbance Rejection Bode Plot');

% 3) Gain and phase margins of C*G
figure;
margin(C * G);
title('Gain and Phase Margins');

%% ========================== STEP RESPONSE ==============================
% Evaluate the time-domain response to a step input (commented out by default).
% figure;
% step(sys);
% title('Step Response');

% Retrieve step-response characteristics (rise time, settling time, etc.).
stepInformation = stepinfo(sys);

%% ====================== SIMULINK RESPONSE SETUP ========================
% Prepare parameters for the Simulink model run.

% Extract transfer function data from the controller
[numC, denomC] = tfdata(C);
numC   = cell2mat(numC);
denomC = cell2mat(denomC);
Kp = C.Kp;
Ki = C.Ki;
Kd = C.Kd;

% Extract transfer function data from the plant
[numG, denomG] = tfdata(G);
numG   = cell2mat(numG);
denomG = cell2mat(denomG);

% Extract transfer function data from the sensor
[numH, denomH] = tfdata(H);
numH   = cell2mat(numH);
denomH = cell2mat(denomH);

% Disturbance setup (for demonstration)
% 'disturbanceSign': +1 => positive speed change, -1 => negative speed change
disturbanceSign = -1;  % Disturbance sign
distTime = 7;          % [s] Time to apply step disturbance
stepGain = 1;          % Magnitude of the step disturbance

% Run the Simulink model ('controlSys.slx') for 15 seconds
simout = sim('controlSys.slx', 'StopTime', "15");
tout = simout.tout;

%% ===================== PROCESS SIMULATION OUTPUT =======================
% Process the logged signals from the Simulink run.

% Optional index shift to align initial time
startInd = 1; 
tout = tout(startInd:end) - tout(startInd);
yout = simout.yout{1}.Values.Data(startInd:end);

% Plot the simulated angular velocity versus time
figure;
plot(tout, yout, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Simulink Response - Angular Velocity vs. Time');
grid on;
