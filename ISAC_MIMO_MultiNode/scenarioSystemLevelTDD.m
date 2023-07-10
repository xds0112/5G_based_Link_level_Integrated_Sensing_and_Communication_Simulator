%% System-Level ISAC Scenario 

% Scenario Generation.

% Author: D.S.Xue, Key Laboratory of Universal Wireless Communications,
% Ministry of Education, BUPT.

%%
clc; close all; clear;
rng('shuffle')

%% Simulation Time
numFrames = 1;    % Number of 5G NR radio frame (10ms)

%% Radar Sensing Parameters
% CFAR detector configuration
Pfa = 1e-9;                   % False alarm rate
estZone = [50 1000; -50 50];  % Estimation zone, [a b; c d] a to b m, c to d m/s

%% gNB Parameters
numCells  = 1;
gNBParams = repmat(struct,numCells,1);

% gNB ID.1
% gNB ISAC services
gNBParams(1).ComService = true;
gNBParams(1).SenService = true;
% gNB communication params
gNBParams(1).CellID           = 1;        % Cell ID
gNBParams(1).CellType         = 'Macro';  % Cell type
gNBParams(1).CarrierFrequency = 3.50e9;   % Carrier frequency (Hz)
gNBParams(1).SCS              = 30;       % 15, 30, 60, 120, 240 (kHz)
gNBParams(1).Bandwidth        = 100;      % Cell bandwidth (MHz)
gNBParams(1).AntSize          = [2 1 2];  % Antenna size in a panel, [rows, columns, polarizations]
gNBParams(1).tddPattern       = ["D" "D" "D" "S" "U" "D" "D" "S" "U" "U"];
gNBParams(1).specialSlot      = [10 2 2]; % Flexible slot configuration (downlink + guard + uplink)
gNBParams(1).numUEs           = 1;        % Numbers of UEs associated
gNBParams(1).SNRdBDL          = 25;       % DL communication SNRs (in dB)
gNBParams(1).SNRdBUL          = 20;       % UL communication SNRs (in dB)
% Radar sensing params 
gNBParams(1).numTargets  = 1;                                              % Number of targets associated
gNBParams(1).RxRCS       = .1*ones(1,gNBParams(1).numTargets);             % Radar cross-section (m^2)
gNBParams(1).Range       = randi([30 300],[1 gNBParams(1).numTargets]);    % Distance (m) (row vecotor)
gNBParams(1).Vel         = randi([-10 10],[1 gNBParams(1).numTargets]);    % Radial velocity (m/s) (row vecotor)
gNBParams(1).MoveDir     = [randi([-60 60],[1 gNBParams(1).numTargets]);             
                            randi([-60 60],[1 gNBParams(1).numTargets])];  % [azimuth; elevation] (°)
gNBParams(1).estAlgorithm = 'MUSIC';                                        % Parameter estimation algorithm, 'FFT', 'MUSIC'
gNBParams(1).Pfa         = Pfa;                                             % False alarm rate
gNBParams(1).cfarEstZone = estZone;                                         % Estimation zone

% gNB ID.2
% ...

% gNB ID.3
% ...

% Update parameters
[gNBParams,algParams,channels,antConfig] = communication.hMultigNBGeneration(gNBParams,numFrames);

%% Launch Simulation
% Flag to set up parallel simulation.
% If 'true', the simulation will be run in parallel mode, which can improve 
% the efficiency of the computation when multiple processors are available.
enableParaSimulation = false;

[gNBEstResults,gNBEstRMSE,gNBComResults] = simulate(gNBParams,algParams,channels,antConfig,enableParaSimulation);

