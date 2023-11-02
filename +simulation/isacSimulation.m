function [gNBEstRMSEs, gNBComResults] = isacSimulation(simuParams)
%% Single gNB-based ISAC Transceiver Simulation
%
% Author: D.S Xue, Key Laboratory of Universal Wireless Communications,
% Ministry of Education, BUPT.

%%
    % Parameters initialization
    numFrames   = simuParams.numFrames;
    gNB         = simuParams.bsParams;
    topoParams  = simuParams.topoParams;
    comChannels = simuParams.chlParams;

    % gNB parameters
    algParams   = gNB.algParams;
    tddPattern  = gNB.tddPattern;
    specialSlot = gNB.specialSlot;
    waveInfo    = gNB.waveformInfo;
    carrier     = gNB.carrier;
    PDSCHs      = gNB.PDSCHs;
    SRSs        = gNB.SRSs;

    % Antenna configurations
    antConfig   = gNB.antConfig;
    bsAntSize   = antConfig.bsTxAntSize;
    ueAntSizes  = antConfig.ueAntSizes;
    
    % Reset random generator for reproducibility
    rng('default')

    % Set up record of data transfer and CSI
    dataState = communication.preProcessing.setupDataTransfer(carrier, numFrames, gNB.numLayers);
    csi = communication.preProcessing.setupCSI(carrier, bsAntSize, ueAntSizes);
    
    % Number of total slots
    nSlots = numFrames*carrier.SlotsPerFrame;

    % SNR values for downlink and uplink communication
    SNRdBDL = 25; % in dB
    SNRdBUL = 20; % in dB
    
    % Radar symbols and waveform in all slots combined
    [radarTxGrid, radarTxWave] = deal([]);
    % Radar estimation parameters
    rdrEstParams = sensing.preProcessing.radarParams(nSlots, carrier, waveInfo, gNB, topoParams);
    cfar         = sensing.detection.cfarConfig(rdrEstParams);
    [gNBEstResults, gNBEstRMSEs, gNBComResults] = deal(NaN);
    
    %% Simulation Loop
    for iSlot = 0:nSlots-1
        % Update the slot index
        carrier.NSlot = iSlot;

        % Schedule UEs for data transmission
        [schedule, PDSCHs] = communication.multiUserSelection(csi, tddPattern, specialSlot, carrier, PDSCHs, bsAntSize, algParams);
    
        % PDSCH transmissions for all UEs scheduled for data
        [txDL, singleLayerTBS] = communication.multiDLTransmit(carrier, PDSCHs(schedule.PDSCH), bsAntSize);
    
        % SRS transmissions for all UEs scheduled for SRS
        [txUL, schedule.SRS] = communication.multiULTransmit(carrier, SRSs);
    
        % Apply communication propagation channels
        [comChannels, rxDL, rxUL] = communication.channelModels.applyMultiUserChannels(tddPattern, specialSlot, carrier, schedule, comChannels, txDL, txUL);
    
        % Apply AWGN
        rxDL = communication.channelModels.applyMultiUserAWGN(carrier, rxDL, SNRdBDL, CombineWaveforms = false);
        rxUL = communication.channelModels.applyMultiUserAWGN(carrier, rxUL, SNRdBUL, CombineWaveforms = true);
    
        % For all UEs scheduled for SRS, estimate CSI and record it
        [H, nVar] = communication.multiULReceive(carrier, SRSs(schedule.SRS), rxUL, algParams);
        csi = communication.updateCSI(csi, carrier, schedule.SRS, H, nVar);
        
        % Communication performance evaluation
        if gNB.comService
            % For all UEs scheduled for data, perform PDSCH reception and record the results
            [TBS, CRC, ~] = communication.multiDLReceive(carrier, PDSCHs(schedule.PDSCH), rxDL, algParams);
            dataState = communication.updateDataTransfer(dataState, carrier, singleLayerTBS, schedule.PDSCH, TBS, CRC);
        end

        % Sensing symbols
        if gNB.senService 
            % Symbols accumulation
            radarTxGrid = cat(2, radarTxGrid, txDL.dlGrid);
            % Wave accumulation
            radarTxWave = cat(1, radarTxWave, txDL.dlWaveform);
        end

    end

    % Communication results
    if gNB.comService
        gNBComResults = communication.summarizeResults(dataState);
        communication.plotResults(dataState, numFrames)
    end

    % Sensing processing
    if gNB.senService
        % Radar active sensing
        radarRxGrid = sensing.monoStaticSensing(radarTxWave, carrier, waveInfo, gNB, rdrEstParams);
        % Radar parameter estimation algorithm
        if strcmp(gNB.estAlgorithm, 'FFT')
            gNBEstResults = sensing.estimation.fft2D(rdrEstParams, cfar, radarRxGrid, radarTxGrid);
        elseif strcmp(gNB.estAlgorithm, 'MUSIC')
            gNBEstResults = sensing.estimation.music2D(rdrEstParams, gNB, radarRxGrid, radarTxGrid);
        end

        % Get estimation RMSEs
        gNBEstRMSEs = sensing.postProcessing.getRMSE(gNBEstResults, rdrEstParams);

        % Plot topology
        networkTopology.plotTopology(gNB, gNBEstResults)
    end
    
end

