function [bsComResults, bsEstResults, bsEstRMSE] = singleBSSimulation(simulationTime, bsParams, topoParams)
%% Single ISAC-BS Simulation

% Author: D.S Xue, Key Laboratory of Universal Wireless Communications,
% Ministry of Education, BUPT.

%%
    % Params initialization
    simLocal    = bsParams;
    tddPattern  = simLocal.tddPattern;
    specialSlot = simLocal.specialSlot;
    waveInfo    = simLocal.waveformInfo;
    carrier     = simLocal.carrier;
    PDSCHs      = simLocal.PDSCHs;
    SRSs        = simLocal.SRSs;
    comChannels = simLocal.comChannelModel;
    bsAntSize   = simLocal.antConfig.bsAntSize;
    ueAntSizes  = simLocal.antConfig.ueAntSizes;
    algParams   = simLocal.algParams;

    % Number of total slots
    nSlots = simulationTime*carrier.SlotsPerFrame;
    
    % Reset random generator for reproducibility
    rng('default')

    % Set up record of data transfer and CSI
    dataState = communication.preProcessing.setupDataTransfer(carrier,simulationTime,simLocal.numLayers);
    csi = communication.preProcessing.setupCSI(carrier,bsAntSize,ueAntSizes);

    % DL and UL SNRs
    snrdBDL = 25;
    snrdBUL = 20;

    % Radar symbols in all slots combined
    rdrTxGrid = [];
    % Radar estimation parameters
    rdrEstParams = sensing.preProcessing.radarParams(nSlots,carrier,waveInfo,simLocal,topoParams);
    cfar         = sensing.dectection.CFAR(rdrEstParams);
    [bsEstResults,bsEstRMSE,bsComResults] = deal(NaN);
    
    %% Simulation Loop
    for iSlot = 0:nSlots-1
        % Update the slot index
        carrier.NSlot = iSlot;

        % Schedule UEs for data transmission
        [schedule,PDSCHs] = communication.scheduler.multiUserSelection(csi,tddPattern,specialSlot,carrier,PDSCHs,bsAntSize,algParams);
    
        % PDSCH transmissions for all UEs scheduled for data
        [txDL,singleLayerTBS] = communication.dlTransmission.multiDLTransmit(carrier,PDSCHs(schedule.PDSCH),bsAntSize);
    
        % SRS transmissions for all UEs scheduled for SRS
        [txUL,schedule.SRS] = communication.ulTransmission.multiULTransmit(carrier,SRSs);
    
        % Apply communication propagation channels
        [comChannels,rxDL,rxUL] = communication.channelModels.applyMultiUserChannels(tddPattern,specialSlot,carrier,schedule,comChannels,txDL,txUL);
    
        % Apply AWGN
        rxDL = communication.channelModels.applyMultiUserAWGN(carrier,rxDL,snrdBDL,CombineWaveforms = false);
        rxUL = communication.channelModels.applyMultiUserAWGN(carrier,rxUL,snrdBUL,CombineWaveforms = false);
    
        % For all UEs scheduled for SRS, estimate CSI and record it
        [H,nVar] = communication.ulTransmission.multiULReceive(carrier,SRSs(schedule.SRS),rxUL,algParams);
        csi = communication.ulTransmission.updateCSI(csi,carrier,schedule.SRS,H,nVar);

        % Sensing process (only when [enableSenEvaluation] is set to 'true')
        if simLocal.senService 

            % Determine the current slot type
            [slotType,~] = communication.scheduler.determineSlotType(tddPattern,specialSlot,carrier);

            if (slotType == "D")

                % Symbols accumulation
                rdrTxGrid = cat(2,rdrTxGrid,txDL.dlGrid);

            elseif (slotType == "S")

                % Radar active sensing
                rdrRxGrid = sensing.monoStaticSensing(rdrTxGrid,carrier,waveInfo,simLocal,rdrEstParams,topoParams);

                % Radar parameter estimation algorithm
                if strcmp(simLocal.estAlgorithm, 'FFT')

                    bsEstResults = sensing.estAlgorithms.FFT(rdrEstParams,cfar,rdrRxGrid,rdrTxGrid);
                    
                elseif strcmp(simLocal.estAlgorithm, 'MUSIC')

                    bsEstResults = sensing.estAlgorithms.MUSIC(rdrEstParams,rdrRxGrid,rdrTxGrid,simLocal);

                end

                % Empty radar Tx & Rx grid
                rdrTxGrid = [];

            end
        end
        
        % Communication performance evaluation (only when [enableComEvaluation] is set to 'true')
        if simLocal.comService
            % For all UEs scheduled for data, perform PDSCH reception and record the results
            [TBS,CRC,~] = communication.dlTransmission.multiDLReceive(carrier,PDSCHs(schedule.PDSCH),rxDL,algParams);
            dataState = communication.dlTransmission.updateDataTransfer(dataState,carrier,singleLayerTBS,schedule.PDSCH,TBS,CRC);
        end

    end
    
    % Communication results of each gNB (only when [enableComEvaluation] is set to 'true')
    if simLocal.comService
        bsComResults = communication.postProcessing.summarizeResults(dataState);
    end

end

