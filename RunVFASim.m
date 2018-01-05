% Object-oriented approach to running VFA simulation
% 
% This script generates all the data required to run the simulations
% "VFA_ActOrder1.slx" and "VFA_ActOrder2.slx", which are
% relative-degree 2 and 3 MIMO VFA simulations, respectively.
% 
% The simulation uses a nonlinear VFA model as the plant, with controllers
% based on linearizations of the (nominal version of the) model at
% different dihedral angles. The adaptive controller is designed according
% to the method described in the paper "Adaptive Output-Feedback Control 
% for a Class of Multi-Input Multi-Output Plants with Arbitrary Relative 
% Degree" by Zheng (Max) Qu et al. These simulation files originate with
% Max's work, but have been continued by Ben Thomsen and other lab members
% since Max's graduation.
% 
% The main steps of the simulation process are:
%   1. Trim the nominal nonlinear VFA model (without actuator dynamics) at 
%       different dihedral angles (find the inputs and states such that 
%       states are not changing) and compute linearized state-space 
%       representation
%   2. Introduce uncertainties and actuator dynamics
%   3. For each trim point (see table_generation_script.m):
%       a. Augment linearized system with actuator dynamics and integral
%           error for command tracking
%       b. Add ficticious inputs to "square-up" the system - to provide
%           SPR properties necessary for adaptive control design
%       c. Compute baseline LQR feedback matrix (K), CRM feedback matrix 
%           (L), along with ouptut-mixing matrix (S) and transformed 
%           coordinates according to method in paper
%   4. Set initial conditions, choose adaptation rates and related gains
%       for simulation, define command trajectory, and run simulation
% 
% With first-order actuator model ("Relative-degree 2"):
% States (x): Airspeed [fps], AOA [deg], Pitch Angle [deg], Pitch Rate [deg/s], 
%             Dihedral [deg], Dihedral Rate [deg/s], Aileron [deg], 
%             Elevator [deg], Dihedral Int Error [ft], Vert Accel Int Error [fps]
%
% With second-order actuator model ("Relative-degree 3"):
% States (x): Airspeed [fps], AOA [deg], Pitch Angle [deg], Pitch Rate [deg/s], 
%             Dihedral [deg], Dihedral Rate [deg/s], Aileron [deg], 
%             Elevator [deg], Aileron Rate [deg/s], Elevator Rate [lbs/s],
%             Dihedral Int Error [ft], Vert Accel Int Error [fps]
% 
% Outputs (y):      Pitch Rate [deg/s], Dihedral Integral Error [deg.s], 
%                   Vertical Accel Integral Error [fps]
%
% Goal is for outputs z to track command r (some notation uses z_cmd)
% Reg. Outputs (z): Dihedral [deg], Vertical Accel. [fps^2]
% 
% Inputs (u):       Aileron [deg], Elevator [deg]
% 
% The use of this simulation requires the Control System Toolbox and also
% Simulink Control Design

clear; clc;

opt.dataPath   = ['data', filesep]; % where to look for/store .mat files 
                                    % (relative to working directory)
opt.adaFlag    = true;              % use adaptive control?
opt.uncertFlag = true;              % uncertainty in dynamics?
opt.actMode    = 'Slow';            % actuator speeds
opt.actOrder   = 2;                 % order of actuator dynamics (1 or 2)
opt.reTrim     = true;              % trim, linearize, recompute controller

vfa = SimVFA(opt);  % initialize and setup the simulation
vfa.runSim();       % run the simulation
vfa.plotSim();      % plot output from the simulation
