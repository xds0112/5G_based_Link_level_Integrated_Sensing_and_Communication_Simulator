classdef gNB < networkElements.networkElementsWithPosition
    %GNB 
    %   
    
    properties

        % Type, 'Macro', 'Micro', ...
        gNBType

        % Height of the antenna (m)
        height = 30

        % Power on a fully allocated grid (dBm)
        txPower = 46

        % Antenna gain (dBi)
        antGain = 25.5

        % Noise figure (dB)
        noiseFigure = 6

        % Antenna temperature (K)
        antTemperature = 290

        % Tx array
        txArray

        % Antenna configurations
        antConfig = struct

        % Algorithmic parameters
        algParams = struct

        % TDD pattern
        tddPattern

        % TDD special slot
        specialSlot

        % OFDM carrier frequency
        carrierFrequency

        % Cell bandwidth
        bandwidth

        % Subcarrier spacing
        scs

        % OFDM carrier configurations
        carrier = nrCarrierConfig;

        % OFDM wave information
        waveformInfo

        % Number of layers
        numLayers

        % PDSCH configuration
        PDSCHs = struct

        % SRS configuration
        SRSs = nrSRSConfig

        % Communication services
        comService = true;

        % Attached UEs
        attachedUEs

        % Communication channel delay profile
        delayProfile = 'CDL-D'

        % Communication channel model
        comChannelModel = nrCDLChannel

        % Pathloss model
        pathloss = nrPathLossConfig

        % Sensing services
        senService = true;

        % Attached targets
        attachedTgts

        % Sensing channel model
        senChannelModel

        % Flase alarm rate
        Pfa = 1e-9

        % 2D-CFAR estimation zone,
        % defined as [a b; c d],
        % a to b meters,
        % c to d meters per second
        cfarEstZone = [50 500; -50 50]

        % Estimation algorithm
        estAlgorithm = 'FFT'

    end
    
    methods
        function obj = gNB()
            %GNB 
            %  
            obj@networkElements.networkElementsWithPosition()
            obj.height = obj.position(3);

        end

        function obj = updateParams(obj,numUEs,tgtParams,tddPattern,specialSlot,antSize)

            obj = obj.determineType;
            obj = obj.generateCarrier;
            obj = obj.getAttachedTargets(tgtParams);
            obj = obj.generatePDSCHs(numUEs);
            obj = obj.generateSRSs(tddPattern,specialSlot);
            obj = obj.getAntennaArray(antSize);
            obj = obj.getAlgoParams;
            obj = obj.getChannelModel;

        end

        function obj = determineType(obj)
            
            if (obj.carrierFrequency > 0.410e9 && obj.carrierFrequency <= 7.625e9)
                obj.gNBType = 'Macro';
            elseif (obj.carrierFrequency > 24.250e9 && obj.carrierFrequency <= 52.600e9)
                obj.gNBType = 'Micro';
            else
                obj.gNBType = 'Not defined in the 3GPP standards';
            end

        end

       function obj = generateCarrier(obj)

            obj.carrier.NCellID           = obj.ID;
            obj.carrier.SubcarrierSpacing = obj.scs;
            obj.carrier.CyclicPrefix      = 'Normal';
            obj.carrier.NSizeGrid         = communication.preProcessing.determinePRB( ...
                                             obj.bandwidth,obj.scs,obj.carrierFrequency);

            nFFT = 2^nextpow2(obj.carrier.NSizeGrid*12); 
            obj.waveformInfo = nrOFDMInfo(obj.carrier,'Nfft',nFFT);

        end

        function obj = getAttachedTargets(obj,tgtParams)

            obj.attachedTgts = tgtParams;

        end

        function obj = generatePDSCHs(obj,numUEs)

            rng('default')
        
            obj.attachedUEs = numUEs;

            % Number of layers for each UE
            obj.numLayers = pow2(randi([0 1],[1 numUEs]));

            % PDSCH configuration
            obj.PDSCHs = communication.preProcessing.multiUserPDSCH(obj.numLayers);

        end
       

        function obj = generateSRSs(obj,tddPattern,specialSlot)

            % SRS configuration
            if(sum(specialSlot) ~= obj.carrier.SymbolsPerSlot)
                error(['specialSlot must contain ' num2str(obj.carrier.SymbolsPerSlot) ' symbols.']);
            elseif strcmp(tddPattern(:),"D") % Full downlink transmission
                obj.SRSs = NaN;
            else
                obj.SRSs = communication.preProcessing.multiUserSRS(tddPattern,specialSlot,obj.carrier,obj.numLayers);
            end

            % Update properties
            obj.tddPattern  = tddPattern;
            obj.specialSlot = specialSlot;

        end

        function obj = getAntennaArray(obj,antSize)

            % Wavelength
            lambda = physconst('LightSpeed')/obj.carrierFrequency;
        
            % Antenna array matrix
            bsAntSize = antSize;
            nTxAnts = prod(bsAntSize);
            
            % Configure the uniform planar array (UPA) or uniform linear array (ULA)
            % based on the sizes of antennas arrays.
            if ~any(bsAntSize == 1)
                % Default antenna elements are [phased.NRAntennaElement]
                obj.txArray = phased.NRRectangularPanelArray('Element',phased.NRAntennaElement, ...
                    'Size',[bsAntSize(1),bsAntSize(2),1,1], 'Spacing',[.5,.5,3,3]*lambda);
            else
                % Configure the transmit antenna elements
                obj.txArray = phased.ULA('Element',phased.IsotropicAntennaElement('BackBaffled',true), ...
                    'NumElements',nTxAnts,'ElementSpacing',.5*lambda);
            end

            % Update antenna configurations
            obj.antConfig.bsAntSize = bsAntSize;

        end

        function obj = getAlgoParams(obj)

            % Algorithmic parameters
            obj.algParams.PrecodingMethod = 'RZF'; % MU-MIMO precoding method: 'BD', 'RZF', 'ZF'
            obj.algParams.ScheduledLayers = obj.numLayers;
            obj.algParams.PerfectChannelEstimator = true;

        end

        function obj = getChannelModel(obj)

            % CDL channel models
            ueAntSizes = 1 + (obj.numLayers.' > [4 2 1]);
            delaySpread = 300e-9;
            maximumDopplerShift = 5;
            obj.comChannelModel = communication.channelModels.multiUserChannels(obj.delayProfile, ...
                obj.carrierFrequency,delaySpread,maximumDopplerShift,obj.antConfig.bsAntSize,ueAntSizes);

            % Update antenna configurations
            obj.antConfig.ueAntSizes = ueAntSizes;

        end

    end


end

