%% Link-Level Full Downlink Radar Sensing Scenario 

% For link-level radar sensing simulation only.

% Author: D.S.Xue, Key Laboratory of Universal Wireless Communications,
% Ministry of Education, BUPT.

%%
clc; close all; clear;
rng('default')
printCopyRight

%% Simulation Time
numFrames = 1; % Number of radio frames (10 ms)

%% Base Station
% Initialize bs
bs = networkElements.baseStation.gNB;
bs.ID               = 1;
bs.position         = [0 0 30];        % in meters
bs.txPower          = 46;              % in dBm
bs.carrierFrequency = 3.50e9;          % in Hz
bs.bandwidth        = 100;             % in MHz
bs.scs              = 60;              % in kHz

% Full downlink transmission
tddPattern  = "D";
specialSlot = [14 0 0];

% BS antenna panel
antSize = [4 1 2];

% Attached UEs
ue = networkElements.ue.basicUE();
ue.ID       = 1;
ue.position = [150 50 1.5];  % in meters
ueParams    = ue;

% Attached targets
tgt = networkElements.target.basicTarget();
tgt.ID       = 1;
tgt.position = [150 50 1.5];  % in meters
tgt.rcs      = 1;             % in meters^2
tgt.velocity = 5;             % in meters per second
tgtParams    = tgt;

% Update bs and UE/target pairs
bs = bs.updateParams(ueParams,tgtParams,tddPattern,specialSlot,antSize);

% Get topology params
topoParams = networkTopology.getTopoParams(bs);

% Generate communication channel models
comChannel = communication.channelModels.cdlModel;
comChannel.carrierFrequency = bs.carrierFrequency;
comChannel.delayProfile     = 'CDL-D';
comChannel.attachedUEs      = ueParams;
comChannel.antConfig        = bs.antConfig;
comChannel.topoParams       = topoParams;
comChannel = comChannel.updateChannel;

%% Launch Simulation
% Params
bsParams = bs;
carrier  = bsParams.carrier;
pdsch    = bsParams.PDSCHs.Config;
pdschExt = bsParams.PDSCHs.Extension;
waveInfo = bsParams.waveformInfo;
channel  = comChannel.model.channel;
nSlots   = numFrames*carrier.SlotsPerFrame;

% Pre-processing for link transmission
[harq,dlsch,newWtx] = preProcessing(pdsch,pdschExt,channel,carrier);

% Downlink grid mapping
nTxAnts = prod(bsParams.antConfig.bsAntSize);
rdrTxGrid = dlTransmit(numFrames,carrier,pdsch,pdschExt,nTxAnts,newWtx,harq,dlsch);

% Radar mono-static detection and estimation
rdrEstParams        = sensing.preProcessing.radarParams(nSlots,carrier,waveInfo,bsParams,topoParams);
cfar                = sensing.dectection.CFAR(rdrEstParams);
rdrRxGrid           = sensing.monoStaticSensing(rdrTxGrid,carrier,waveInfo,bsParams,rdrEstParams,topoParams);
gNBEstResults.FFT   = sensing.estAlgorithms.FFT(rdrEstParams,cfar,rdrRxGrid,rdrTxGrid);
gNBEstResults.MUSIC = sensing.estAlgorithms.MUSIC(rdrEstParams,bsParams,rdrRxGrid,rdrTxGrid);

%% Local Functions
% used in link-level verifications only

function printCopyRight

    currentTime = datetime('now');

    fprintf(['%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%' ...
        '%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n']);
    fprintf('Copyright (C) 2023 Beijing University of Posts and Telecommunications\n');
    fprintf('All rights reserved.\n');
    fprintf('\n');
    fprintf('Link-level ISAC Verification Simulator\n');
    fprintf('Author: Dongsheng Xue\n');
    fprintf('Date: %s\n', char(currentTime));
    fprintf(['%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%' ...
        '%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n']);

end

function [harq,dlsch,newWtx] = preProcessing(pdsch,pdschExt,comChannel,carrier)

    % Set up HARQ
    harq = communication.HARQEntity(0:15,[0 2 3 1],pdsch.NumCodewords);
    % Create DLSCH encoder object and decoder object
    dlsch.encodeDLSCH = nrDLSCH('MultipleHARQProcesses',true,'TargetCodeRate',pdschExt.TargetCodeRate);
    dlsch.decodeDLSCH = nrDLSCHDecoder('MultipleHARQProcesses',true,'TargetCodeRate',pdschExt.TargetCodeRate);
    % Get precoding weight
    estChannelGrid = getInitialChannelEstimate(comChannel,carrier);
    newWtx = getPrecodingMatrix(pdsch.PRBSet,pdsch.NumLayers,estChannelGrid);

end

function rdrTxGrid = dlTransmit(numFrames,carrier,pdsch,pdschExt,nTxAnts,newWtx,harq,dlsch)

    % DLSCH encoder object and decoder object    
    encodeDLSCH = dlsch.encodeDLSCH;
    decodeDLSCH = dlsch.decodeDLSCH;

    % Radar Tx and Rx symbols in all slots combined
    rdrTxGrid = [];
    
    % Number of total slots
    nSlots = numFrames*carrier.SlotsPerFrame;

    % Slot loop
    for iSlot = 0:nSlots-1
        % Update the slot index
        carrier.NSlot = iSlot;
    
        % PDSCH generation
        [pdschIndices,pdschInfo] = nrPDSCHIndices(carrier,pdsch);    

        % Calculate transport block sizes
        Xoh_PDSCH = 0;
        trBlkSizes = nrTBS(pdsch.Modulation,pdsch.NumLayers,numel(pdsch.PRBSet), ...
            pdschInfo.NREPerPRB,pdschExt.TargetCodeRate,Xoh_PDSCH);

        % Get new transport blocks and flush decoder soft buffer, as required
        for cwIdx = 1:pdsch.NumCodewords
            if harq.NewData(cwIdx)
                % Create and store a new transport block for transmission
                trBlk = randi([0 1],trBlkSizes(cwIdx),1);
                setTransportBlock(encodeDLSCH,trBlk,cwIdx-1,harq.HARQProcessID);

                % If the previous RV sequence ends without successful
                % decoding, flush the soft buffer
                if harq.SequenceTimeout(cwIdx)
                    resetSoftBuffer(decodeDLSCH,cwIdx-1,harq.HARQProcessID);
                end
            end
        end

        % DL-SCH encoding
        codedTrBlock = encodeDLSCH(pdsch.Modulation,pdsch.NumLayers,pdschInfo.G, ...
            harq.RedundancyVersion,harq.HARQProcessID);

        % Generate PDSCH symbols
        pdschSymbols = nrPDSCH(carrier,pdsch,codedTrBlock);

        % Precode the PDSCH symbols
        precodingWeights = newWtx;
        pdschSymbolsPrecoded = pdschSymbols*precodingWeights;

        % PDSCH DMRS generation
        dmrsSymbols = nrPDSCHDMRS(carrier,pdsch);
        dmrsIndices = nrPDSCHDMRSIndices(carrier,pdsch);

        % PDSCH precoding and mapping
        pdschGrid = nrResourceGrid(carrier,nTxAnts);
        [~,pdschAntIndices] = nrExtractResources(pdschIndices,pdschGrid);
        pdschGrid(pdschAntIndices) = pdschSymbolsPrecoded;

        % PDSCH DMRS precoding and mapping
        for p = 1:size(dmrsSymbols,2)
            [~,dmrsAntIndices] = nrExtractResources(dmrsIndices(:,p),pdschGrid);
            pdschGrid(dmrsAntIndices) = pdschGrid(dmrsAntIndices) + dmrsSymbols(:,p)*precodingWeights(p,:);
        end

        % Downlink symbols accumulation
        rdrTxGrid = cat(2,rdrTxGrid,pdschGrid);

     end

end

function estChannelGrid = getInitialChannelEstimate(channel,carrier)
% Obtain an initial channel estimate for calculating the precoding matrix.
% This function assumes a perfect channel estimate

    % Clone of the channel
    chClone = channel.clone();
    chClone.release();

    % No filtering needed to get channel path gains
    chClone.ChannelFiltering = false;    
    
    % Get channel path gains
    [pathGains,sampleTimes] = chClone();
    
    % Perfect timing synchronization
    pathFilters = getPathFilters(chClone);
    offset = nrPerfectTimingEstimate(pathGains,pathFilters);
    
    % Perfect channel estimate
    estChannelGrid = nrPerfectChannelEstimate(carrier,pathGains,pathFilters,offset,sampleTimes);
end

function wtx = getPrecodingMatrix(PRBSet,NLayers,hestGrid)
% Calculate precoding matrix given an allocation and a channel estimate
    
    % Allocated subcarrier indices
    allocSc = (1:12)' + 12*PRBSet(:).';
    allocSc = allocSc(:);
    
    % Average channel estimate
    [~,~,R,P] = size(hestGrid);
    estAllocGrid = hestGrid(allocSc,:,:,:);
    Hest = permute(mean(reshape(estAllocGrid,[],R,P)),[2 3 1]);
    
    % SVD decomposition
    [~,~,V] = svd(Hest);
    
    wtx = V(:,1:NLayers).';
    wtx = wtx/sqrt(NLayers); % Normalize by NLayers
end