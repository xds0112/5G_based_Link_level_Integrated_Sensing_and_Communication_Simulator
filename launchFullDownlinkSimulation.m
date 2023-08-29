%% Simulator Launcher File
% launches link level scenario

%%
clc; close all; clear;

% Print copyright
tools.printCopyright

%% Launch simulation
% Get simulation parameters
simuParams = scenarios.scenarioFullDownlink();

% Parameters
numFrames  = simuParams.numFrames;
bsParams   = simuParams.bsParams;
topoParams = simuParams.topoParams;
chlParams  = simuParams.chlParams;

% gNB parameters
carrier  = bsParams.carrier;
pdsch    = bsParams.PDSCHs.Config;
pdschExt = bsParams.PDSCHs.Extension;
waveInfo = bsParams.waveformInfo;

% Channel parameters
channel = chlParams.channel;

% Time parameters
numSlots = numFrames*carrier.SlotsPerFrame;

% HARQ, DLSCH and precoding matrix set up for link transmission
[harq, dlsch, newWtx] = communication.preProcessing.fullDownlinkTransmissionSetup(pdsch, pdschExt, channel, carrier);

% Downlink grid mapping and transmitting
nTxAnts = prod(bsParams.antConfig.bsAntSize);
rdrTxGrid = communication.fullDownlinkTransmit(numFrames, carrier, pdsch, pdschExt, nTxAnts, newWtx, harq, dlsch);

% Radar mono-static detection and estimation
rdrEstParams     = sensing.preProcessing.radarParams(numSlots, carrier, waveInfo, bsParams, topoParams);
cfar             = sensing.detection.cfarConfig(rdrEstParams);
rdrRxGrid        = sensing.monoStaticSensing(rdrTxGrid, carrier, waveInfo, bsParams, rdrEstParams, topoParams);
estResults.FFT   = sensing.estimation.fft2D(rdrEstParams, cfar, rdrRxGrid, rdrTxGrid);
% estResults.MUSIC = sensing.estimation.music2D(rdrEstParams, bsParams, rdrRxGrid, rdrTxGrid);

% Plot topology
networkTopology.plotTopology(bsParams, estResults.FFT)

% Get estimation RMSEs
estRMSE = sensing.postProcessing.getRMSE(estResults.FFT, rdrEstParams);

