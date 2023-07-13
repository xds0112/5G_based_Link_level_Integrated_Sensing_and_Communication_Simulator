%% System-Level ISAC Scenario 

% Scenario Generation.

% Author: D.S.Xue, Key Laboratory of Universal Wireless Communications,
% Ministry of Education, BUPT.

%%
clc; close all; clear;
rng('default')

%% Simulation Time
numFrames = 1; % Number of radio frames (10 ms)

%% Base Station 1
% Initialize bs
bs(1) = networkElements.baseStation.gNB;
bs(1).ID               = 1;
bs(1).position         = [0 0 30];        % in meters
bs(1).txPower          = 46;              % in dBm
bs(1).carrierFrequency = 3.50e9;          % in Hz
bs(1).bandwidth        = 100;             % in MHz
bs(1).scs              = 60;              % in kHz

% TDD mode
tddPattern  = ["D" "D" "D" "S" "U" "D" "D" "S" "U" "U"];
specialSlot = [10 2 2];

% Antenna panel
antSize = [2 1 2];

% Attached UEs
numUEs = 1;

% Attached targets
tgt(1) = networkElements.target.basicTarget();
tgt(1).ID       = 1;
tgt(1).position = [50 50 1.5];  % in meters
tgt(1).rcs      = 1;            % in meters^2
tgt(1).velocity = 5;            % in meters per second

tgt(2) = networkElements.target.basicTarget(); 
tgt(2).ID       = 2;
tgt(2).position = [25 70 1.5];  % in meters
tgt(2).rcs      = 1;            % in meters^2
tgt(2).velocity = -5;           % in meters per second

tgtParams = tgt;

% Update bs and UE/target pairs
bs(1) = bs(1).updateParams(numUEs, tgtParams, tddPattern, specialSlot, antSize);


%% Launch Simulation
% Simulation params
bsParams = bs;
topologyParams = networkTopology.getTopoParams(bsParams);

% Flag to set up parallel simulation.
% If 'true', the simulation will be run in parallel mode, which can improve 
% the efficiency of the computation when multiple processors are available.
enableParaSimulation = false;

[bsComResults,bsEstResults,bsEstRMSE] = simulate(numFrames,bsParams,topologyParams,enableParaSimulation);

