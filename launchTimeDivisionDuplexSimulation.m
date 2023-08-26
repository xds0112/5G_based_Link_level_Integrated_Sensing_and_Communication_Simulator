%% Simulator Launcher File
% launches time division duplex (TDD) scenario

%%
clc; close all; clear;

% Print copyright
tools.printCopyright

%% Launch Simulation
% Get simulation parameters
simuParams = scenarios.scenarioTimeDivisionDuplex();

% Invoke the ISAC simulator
[senRMSEs, comResults] = simulation.isacSimulation(simuParams);
