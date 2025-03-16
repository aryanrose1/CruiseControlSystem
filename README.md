# Automotive Cruise Control System

This repository demonstrates an **automotive cruise control system** built with **MATLAB** and **Simulink**. The system uses a **DC motor** to drive the vehicle’s wheels and a **PID controller** to maintain a desired speed (angular velocity). It is designed to handle disturbances such as changes in load or road slope.

1. **MATLAB Script (MATLAB_Script.m)**  
   - Defines system parameters (motor, vehicle dynamics, tachometer).  
   - Sets a PID controller with user-specified gains.  
   - Builds transfer functions and performs analysis (root locus, Bode plots).  
   - Prepares data for Simulink and runs the simulation to validate performance.  

2. **Simulink Model (CruiseControlMod.slx or controlSys.slx)**  
   - Represents the DC motor and vehicle inertia in a block diagram.  
   - Implements a PID controller to regulate speed.  
   - Includes a disturbance input for testing robustness.  
   - Logs simulation output (angular velocity) for post-processing and plotting.  
