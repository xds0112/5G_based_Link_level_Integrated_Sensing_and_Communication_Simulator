function [gNBParams,algParams,channels,antConfig] = hMultigNBGeneration(gNBParams,numFrames)
% gNB Parameters Generation

% Author: D.S.Xue, Key Laboratory of Universal Wireless Communications,
% Ministry of Education, BUPT.

%%
for i = 1:length(gNBParams)
%%
    gNBParams(i).NFrames = numFrames; % Simulation time

    %% gNB configuration
    gNBParams(i).TxHeight         = 30;   % Height of the antenna (m)
    gNBParams(i).TxPower          = 46;   % Power on a fully allocated grid (dBm)
    gNBParams(i).AntGain          = 25.5; % Antenna gain (dBi)
    gNBParams(i).RxNoiseFigure    = 6;    % Noise figure (dB)
    gNBParams(i).RxAntTemperature = 290;  % Antenna temperature (K)

    %% Carrier
    % Waveform numerology
    gNBParams(i).Carrier                   = nrCarrierConfig;
    gNBParams(i).Carrier.NCellID           = gNBParams(i).CellID; % Cell ID
    gNBParams(i).Carrier.SubcarrierSpacing = gNBParams(i).SCS;    % Subcarrier spacing
    gNBParams(i).Carrier.CyclicPrefix      = 'Normal';            % 'Normal' or 'Extended' (Extended CP is for 60 kHz SCS only) 
    gNBParams(i).Carrier.NSizeGrid         = communication.hDeterminePRB(gNBParams(1).Bandwidth,...
                                                     gNBParams(1).SCS,gNBParams(1).CarrierFrequency); % Resource grid   
    carrier = gNBParams(i).Carrier;

    %% UE Configuration
    % Reset random generator for reproducibility
    rng('default')

    % Number of layers for each UE
    numLayers = pow2(randi([0 1],[1 gNBParams(i).numUEs]));

    % PDSCH configuration
    PDSCHs = communication.hMultiUserPDSCH(numLayers);

    % SRS configuration
    if(sum(gNBParams(i).specialSlot) ~= carrier.SymbolsPerSlot)
        error(['specialSlot must contain ' num2str(carrier.SymbolsPerSlot) ' symbols.']);
    elseif strcmp(gNBParams(i).tddPattern(:),"D") % Full downlink transmission
        SRSs = NaN;
    else
        SRSs = communication.hMultiUserSRS(gNBParams(i).tddPattern,gNBParams(i).specialSlot,carrier,numLayers);
    end

    % Assignment
    gNBParams(i).NLayers = numLayers;
    gNBParams(i).PDSCHs  = PDSCHs; 
    gNBParams(i).SRSs    = SRSs;

    % Algorithmic parameters
    algParams = struct;
    algParams.PrecodingMethod = 'RZF'; % MU-MIMO precoding method: 'BD', 'RZF', 'ZF'
    algParams.ScheduledLayers = 2;
    algParams.PerfectChannelEstimator = true;

    %% Antenna Array
    % Physical parameters
    c = physconst('LightSpeed');                  % Set the propagation speed
    lambda = c/gNBParams(i).CarrierFrequency;     % Calculate wavelength

    % Antenna array matrix
    bsAntSize = gNBParams(i).AntSize;
    gNBParams(i).NTxAnts = prod(bsAntSize);
    
    % Configure the uniform planar array (UPA) or uniform linear array (ULA)
    % based on the sizes of antennas arrays.
    if ~any(bsAntSize == 1)
        % Default antenna elements are [phased.NRAntennaElement]
        gNBParams(i).txArray = phased.NRRectangularPanelArray('Element',phased.NRAntennaElement, ...
            'Size',[bsAntSize(1),bsAntSize(2),1,1], 'Spacing',[.5,.5,3,3]*lambda);
    else
        % Configure the transmit antenna elements
        gNBParams(i).txArray = phased.ULA('Element',phased.IsotropicAntennaElement('BackBaffled',true), ...
            'NumElements',gNBParams(i).NTxAnts,'ElementSpacing',.5*lambda);
    end

    antConfig.bsAntSize = bsAntSize;

    %% CDL Channel Models
    ueAntSizes = 1 + (numLayers.' > [4 2 1]);
    antConfig.ueAntSizes = ueAntSizes;

    delayProfile = 'CDL-D';
    delaySpread  = 300e-9;
    maximumDopplerShift = 5;
    channels = communication.hMultiUserChannels(delayProfile,gNBParams(i).CarrierFrequency,delaySpread,maximumDopplerShift,bsAntSize,ueAntSizes);

    %% Waveform Info
    nFFT = 2^nextpow2(carrier.NSizeGrid*12); 
    gNBParams(i).waveformInfo = nrOFDMInfo(carrier,'Nfft',nFFT);

end

end
