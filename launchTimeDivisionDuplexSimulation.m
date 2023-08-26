%% Simulator Launcher File
% launches time division duplex (TDD) scenario

%%
clc; close all; clear;


%% Launch Simulation
% Get simulation parameters
simuParams = scenarios.scenarioTimeDivisionDuplex();

% Invoke the ISAC simulator
[senRMSEs, comResults] = simulation.isacSimulation(simuParams);

% Plot topology
networkTopology.plotTopology(simuParams.bsParams, senResults)
