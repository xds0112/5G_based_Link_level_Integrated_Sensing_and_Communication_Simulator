function simuParams = scenarioFullDownlink()
    % Full Downlink Radar Sensing Scenario 
    
    % For link-level radar sensing simulation only
    
    % Author: D.S.Xue, Key Laboratory of Universal Wireless Communications,
    % Ministry of Education, BUPT.
    
    %%
    rng('default')
    
    %% Simulation Time
    numFrames = 1; % Number of radio frames (10 ms)
    
    %% Base Station
    % Initialize bs
    bs = networkElements.baseStation.gNB;
    bs.ID               = 1;
    bs.position         = [0 0 10];         % in meters
    bs.txPower          = 46;               % in dBm
    bs.carrierFrequency = 28.50e9;          % in Hz
    bs.bandwidth        = 100;              % in MHz
    bs.scs              = 60;               % in kHz
    bs.tddPattern       = "D";              % full downlink
    bs.specialSlot      = [14 0 0];         % full downlink symbols
    bs.antSize          = [8 1 2];          % antenna panel
    bs.Pfa              = 1e-9;             % false alarm rate
    bs.cfarEstZone      = [50 500; -20 20]; % [a b; c d], a to b m, c to d m/s
    
    % Attached UEs
    ue(1) = networkElements.ue.basicUE();
    ue(1).ID       = 1;
    ue(1).position = [100 100 1.5];  % in meters
    ueParams       = ue;
    
    % Attached targets
    tgt(1) = networkElements.target.basicTarget();
    tgt(1).ID       = 1;
    tgt(1).position = [100 100 1.5];  % in meters
    tgt(1).rcs      = 1;              % in meters^2
    tgt(1).velocity = 5;              % in meters per second
    tgtParams       = tgt;
    
    % Update bs and UE/target pairs
    bs.attachedUEs  = ueParams;
    bs.attachedTgts = tgtParams;
    
    % Get topology params
    topoParams = networkTopology.getTopoParams(bs);
    
    % Generate communication channel models
    comChannel = communication.channelModels.cdlModel;
    comChannel.carrierFrequency = bs.carrierFrequency;
    comChannel.delayProfile     = 'CDL-D';
    comChannel.attachedUEs      = ueParams;
    comChannel.antConfig        = bs.antConfig;
    comChannel.topoParams       = topoParams;

    %% Simulation Params
    simuParams.numFrames  = numFrames;
    simuParams.bsParams   = bs;
    simuParams.topoParams = topoParams;
    simuParams.chlParams  = comChannel.models;
    
end
