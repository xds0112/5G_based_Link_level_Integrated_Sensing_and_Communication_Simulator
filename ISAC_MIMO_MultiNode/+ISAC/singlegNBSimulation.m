function [gNBEstResults,gNBEstRMSE,gNBComResults] = singlegNBSimulation(gNB,algParams,channels,antConfig)
%% Single ISAC-gNB Simulation

% Author: D.S Xue, Key Laboratory of Universal Wireless Communications,
% Ministry of Education, BUPT.

%%
    % Params initialization
    simLocal    = gNB;
    tddPattern  = simLocal.tddPattern;
    specialSlot = simLocal.specialSlot;
    waveInfo    = simLocal.waveformInfo;
    carrier     = simLocal.Carrier;
    PDSCHs      = simLocal.PDSCHs;
    SRSs        = simLocal.SRSs;
    comChannels = channels;
    bsAntSize   = antConfig.bsAntSize;
    ueAntSizes  = antConfig.ueAntSizes;
    
    % Reset random generator for reproducibility
    rng('default')

    % Set up record of data transfer and CSI
    dataState = communication.setupDataTransfer(carrier,simLocal.NFrames,simLocal.NLayers);
    csi = communication.setupCSI(carrier,bsAntSize,ueAntSizes);

    % Radar symbols in all slots combined
    rdrTxGrid = [];
    % Radar estimation parameters
    rdrEstParams = sensing.hRadarParams(carrier,waveInfo,simLocal);
    cfar         = sensing.hRadarCFAR(rdrEstParams);
    [gNBEstResults,gNBEstRMSE,gNBComResults] = deal(NaN);

    % Number of total slots
    nSlots = simLocal.NFrames*carrier.SlotsPerFrame;
    
    %% Simulation Loop
    for iSlot = 0:nSlots-1
        % Update the slot index
        carrier.NSlot = iSlot;

        % Schedule UEs for data transmission
        [schedule,PDSCHs] = communication.hMultiUserSelection(csi,tddPattern,specialSlot,carrier,PDSCHs,bsAntSize,algParams);
    
        % PDSCH transmissions for all UEs scheduled for data
        [txDL,singleLayerTBS] = communication.hMultiDLTransmit(carrier,PDSCHs(schedule.PDSCH),bsAntSize);
    
        % SRS transmissions for all UEs scheduled for SRS
        [txUL,schedule.SRS] = communication.hMultiULTransmit(carrier,SRSs);
    
        % Apply communication propagation channels
        [comChannels,rxDL,rxUL] = communication.hApplyMultiUserChannels(tddPattern,specialSlot,carrier,schedule,comChannels,txDL,txUL);
    
        % Apply AWGN
        rxDL = communication.hApplyMultiUserAWGN(carrier,rxDL,simLocal.SNRdBDL,CombineWaveforms = false);
        rxUL = communication.hApplyMultiUserAWGN(carrier,rxUL,simLocal.SNRdBUL,CombineWaveforms = false);
    
        % For all UEs scheduled for SRS, estimate CSI and record it
        [H,nVar] = communication.hMultiULReceive(carrier,SRSs(schedule.SRS),rxUL,algParams);
        csi = communication.updateCSI(csi,carrier,schedule.SRS,H,nVar);

        % Sensing process (only when [enableSenEvaluation] is set to 'true')
        if simLocal.SenService 
            % Determine the current slot type
            [slotType,~] = communication.hDetermineSlotType(tddPattern,specialSlot,carrier);
            if (slotType == "D")
                % Tx precoding weights
                for iue = 1:length(PDSCHs)
                    wtx = PDSCHs(iue).Extension.W;
                end
                % Symbols accumulation
                rdrTxGrid = cat(2,rdrTxGrid,txDL.dlGrid);
            elseif (slotType == "S")
                % Radar active sensing
                rdrRxGrid = sensing.activeSensing(rdrTxGrid,carrier,waveInfo,simLocal,rdrEstParams);
                % Radar parameter estimation algorithm
                if strcmp(simLocal.estAlgorithm, 'FFT')
                    gNBEstResults = sensing.hRadar3DFFT(rdrEstParams,cfar,rdrRxGrid,rdrTxGrid,simLocal.txArray,wtx);
                elseif strcmp(simLocal.estAlgorithm, 'MUSIC')
                    gNBEstResults = sensing.hRadarMUSIC(rdrEstParams,rdrRxGrid,rdrTxGrid,simLocal);
                end
                % Empty radar Tx & Rx grid
                rdrTxGrid = [];
            end
        end
        
        % Communication performance evaluation (only when [enableComEvaluation] is set to 'true')
        if simLocal.ComService
            % For all UEs scheduled for data, perform PDSCH reception and record the results
            [TBS,CRC,~] = communication.hMultiDLReceive(carrier,PDSCHs(schedule.PDSCH),rxDL,algParams);
            dataState = communication.updateDataTransfer(dataState,carrier,singleLayerTBS,schedule.PDSCH,TBS,CRC);
        end

    end
    
    % Communication results of each gNB (only when [enableComEvaluation] is set to 'true')
    if simLocal.ComService
        gNBComResults = communication.summarizeResults(dataState);
    end

end

