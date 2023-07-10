%% Link-Level Full Downlink Radar Sensing Scenario 

% For link-level radar sensing simulation only.

% Author: D.S.Xue, Key Laboratory of Universal Wireless Communications,
% Ministry of Education, BUPT.

%%
clc; close all; clear;
rng('default')

%% Communication Parameters
numFrames = .2;    % Number of 5G NR radio frame (10ms)

% DL and UL communication SNRs (in dB)
SNRdBDL = 25;   
SNRdBUL = 20;   

% TDD pattern
tddPattern  = "D";      % Full downlink transimission
specialSlot = [14 0 0]; % Flexible slot configuration (downlink + guard + uplink)

%% gNB Parameters
numgNB = 1;             % Number of gNBs
gNBParams = repmat(struct,numgNB,1);

% gNB ID.1
% gNB communication params
gNBParams(1).CellID           = 1;       % Cell ID
gNBParams(1).CellType         = 'Macro'; % Cell type
gNBParams(1).CarrierFrequency = 3.50e9;  % Carrier frequency (Hz)
gNBParams(1).SCS              = 30;      % 15, 30, 60, 120, 240 (kHz)
gNBParams(1).Bandwidth        = 100;     % Cell bandwidth (MHz)
gNBParams(1).AntSize          = [4 1 2];  % Antenna size in a panel, [rows, columns, polarizations]
gNBParams(1).tddPattern       = tddPattern;
gNBParams(1).specialSlot      = specialSlot;
gNBParams(1).numUEs           = 1;       % Number of UEs associated
gNBParams(1).SNRdBDL          = SNRdBDL;
gNBParams(1).SNRdBUL          = SNRdBUL;
gNBParams(1).TxHeight         = 30;      % Height of the antenna (m)
gNBParams(1).TxPower          = 46;      % Power on a fully allocated grid (dBm)
gNBParams(1).AntGain          = 25.5;    % Antenna gain (dBi)
% Radar sensing params 
gNBParams(1).numTargets  = 2;                          % Number of targets associated
gNBParams(1).RxRCS       = [10 15];                    % Radar cross-section (m^2)
gNBParams(1).Range       = [100 150];                  % Distance (m) (row vecotor)
gNBParams(1).Vel         = [5 -10];                    % Radial velocity (m/s) (row vecotor)
gNBParams(1).MoveDir     = [-30 60; 30 30];            % [azimuth; elevation] (°)
gNBParams(1).Pfa         = 1e-9;                       % False alarm rate
gNBParams(1).cfarEstZone = [50 500; -50 50];           % Estimation zone, [a b; c d] a to b m, c to d m/s

% Update parameters
[gNBParams,~,channels,antConfig] = communication.hMultigNBGeneration(gNBParams,numFrames);

%% Launch Simulation
% Params
carrier    = gNBParams.Carrier;
pdsch      = gNBParams.PDSCHs.Config;
pdschExt   = gNBParams.PDSCHs.Extension;
waveInfo   = gNBParams.waveformInfo;
comChannel = channels.channel;

% HARQ
harq = communication.HARQEntity(0:15,[0 2 3 1],pdsch.NumCodewords);
% Create DLSCH encoder object and decoder object
dlsch.encodeDLSCH = nrDLSCH('MultipleHARQProcesses',true,'TargetCodeRate',pdschExt.TargetCodeRate);
dlsch.decodeDLSCH = nrDLSCHDecoder('MultipleHARQProcesses',true,'TargetCodeRate',pdschExt.TargetCodeRate);
% Get precoding weight
estChannelGrid = getInitialChannelEstimate(comChannel,carrier);
newWtx = getPrecodingMatrix(pdsch.PRBSet,pdsch.NumLayers,estChannelGrid);
pdschExt.W = newWtx;

% Downlink grid mapping
rdrTxGrid = dlTransmit(numFrames,carrier,pdsch,pdschExt,gNBParams.NTxAnts,newWtx,harq,dlsch);

% Radar active sensing and estimation
rdrEstParams        = sensing.hRadarParams(carrier,waveInfo,gNBParams);
cfar                = sensing.hRadarCFAR(rdrEstParams);
rdrRxGrid           = sensing.activeSensing(rdrTxGrid,carrier,waveInfo,gNBParams,rdrEstParams);
gNBEstResults.FFT   = sensing.hRadar3DFFT(rdrEstParams,cfar,rdrRxGrid,rdrTxGrid,gNBParams.txArray,pdschExt.W);
gNBEstResults.MUSIC = sensing.hRadarMUSIC(rdrEstParams,rdrRxGrid,rdrTxGrid,gNBParams);

%% Local Function
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