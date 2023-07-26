%% Simulator Launcher File
% launches link level scenario

%%
clc; close all; clear;

%% Launch simulation
% Parameters
simuParams = scenarios.scenarioLinkLevel();
numFrames  = simuParams.numFrames;
bsParams   = simuParams.bsParams;
topoParams = simuParams.topoParams;
chlParams  = simuParams.chlParams;
carrier  = bsParams.carrier;
pdsch    = bsParams.PDSCHs.Config;
pdschExt = bsParams.PDSCHs.Extension;
waveInfo = bsParams.waveformInfo;
channel  = chlParams;
nSlots   = numFrames*carrier.SlotsPerFrame;

% HARQ, DLSCH and precoding matrix set up for link transmission
[harq, dlsch, newWtx] = communication.preProcessing.transmissionSetup(pdsch, pdschExt, channel, carrier);

% Downlink grid mapping an transmitting
nTxAnts = prod(bsParams.antConfig.bsAntSize);
rdrTxGrid = communication.dlTransmit(numFrames, carrier, pdsch, pdschExt, nTxAnts, newWtx, harq, dlsch);

% Radar mono-static detection and estimation
rdrEstParams     = sensing.preProcessing.radarParams(nSlots, carrier, waveInfo, bsParams, topoParams);
cfar             = sensing.dectection.cfarConfig(rdrEstParams);
rdrRxGrid        = sensing.monoStaticSensing(rdrTxGrid, carrier, waveInfo, bsParams, rdrEstParams, topoParams);
estResults.FFT   = sensing.estimation.fft2D(rdrEstParams, cfar, rdrRxGrid, rdrTxGrid);
estResults.MUSIC = sensing.estimation.music(rdrEstParams, bsParams, rdrRxGrid, rdrTxGrid);

