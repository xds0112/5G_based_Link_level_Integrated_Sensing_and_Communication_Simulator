classdef gNB < networkElements.networkElementsWithPosition
    %GNB 
    %   next generation nodeB / 5G NR base station
    
    properties

        % Height of the antenna (m)
        height = 30

        % Power on a fully allocated grid (dBm)
        txPower = 46

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

        % Communication services
        comService = true

        % Attached UEs
        attachedUEs

        % Sensing services
        senService = true

        % Attached targets
        attachedTgts

        % Antenna size
        antSize

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

    properties (Dependent)
        % Type, 'Macro', 'Micro', ...
        type

        % Tx array
        txArray

        % Antenna configurations
        antConfig

        % OFDM carrier configurations
        carrier

        % OFDM wave information
        waveformInfo

        % Number of layers for each UE
        numLayers

        % PDSCH configuration for each UE
        PDSCHs

        % SRS configuration for each UE
        SRSs

        % Algorithmic parameters
        algParams
    end
    
    methods
        function obj = gNB()
            %GNB 
            %  
            obj@networkElements.networkElementsWithPosition()
            obj.height = obj.position(3);

        end

        function type = get.type(obj)
            
            if (obj.carrierFrequency > 0.410e9 && obj.carrierFrequency <= 7.625e9)
                type = 'Macro';
            elseif (obj.carrierFrequency > 24.250e9 && obj.carrierFrequency <= 52.600e9)
                type = 'Micro';
            else
                type = 'Not defined in the 3GPP standards';
            end

        end

        function carrier = get.carrier(obj)

            carrier = nrCarrierConfig;
            carrier.NCellID           = obj.ID;
            carrier.SubcarrierSpacing = obj.scs;
            carrier.CyclicPrefix      = 'Normal';
            carrier.NSizeGrid         = communication.preProcessing.determinePRB(obj.bandwidth, ...
                                              obj.scs, obj.carrierFrequency);

        end

        function info = get.waveformInfo(obj)

            nFFT = 2^nextpow2(obj.carrier.NSizeGrid*12); 
            info = nrOFDMInfo(obj.carrier,'Nfft',nFFT);

        end

        function nLayers = get.numLayers(obj)

            rng('default')

            % Number of UEs
            numUEs = numel(obj.attachedUEs);

            % Number of layers for each UE, 
            % 1 or 2
            nLayers = pow2(randi([0 1], [1 numUEs]));

        end

        function pdschConfig = get.PDSCHs(obj)

            % PDSCH configuration
            pdschConfig = communication.preProcessing.multiUserPDSCH(obj.numLayers);

        end

        function srsConfig = get.SRSs(obj)

            % SRS configuration
            if(sum(obj.specialSlot) ~= obj.carrier.SymbolsPerSlot)
                error(['specialSlot must contain ' num2str(obj.carrier.SymbolsPerSlot) ' symbols.']);
            elseif strcmp(obj.tddPattern(:),"D") % Full downlink transmission
                srsConfig = NaN;
            else
                srsConfig = communication.preProcessing.multiUserSRS(obj.tddPattern, obj.specialSlot, obj.carrier, obj.numLayers);
            end

        end

        function ant = get.txArray(obj)

            % Wavelength
            lambda = physconst('LightSpeed')/obj.carrierFrequency;
        
            % Antenna array matrix
            bsAntSize = obj.antSize;
            nTxAnts = prod(bsAntSize);
            
            % Configure the uniform planar array (UPA) or uniform linear array (ULA)
            % based on the sizes of antennas arrays.
            if ~any(bsAntSize == 1)
                % Default antenna elements are [phased.NRAntennaElement]
                ant = phased.NRRectangularPanelArray('ElementSet', {phased.NRAntennaElement, phased.NRAntennaElement}, ...
                    'Size', [bsAntSize(1),bsAntSize(2),1,1], 'Spacing', [.5,.5,3,3]*lambda);
            else
                % Configure the transmit antenna elements
                ant = phased.ULA('Element', phased.IsotropicAntennaElement('BackBaffled',true), ...
                    'NumElements', nTxAnts, 'ElementSpacing', .5*lambda);
            end

        end

        function antConfig = get.antConfig(obj)

            antConfig = struct;
            % Update antenna configurations
            %
            % Number of Layers | UE Antenna Array Size
            %        1         |        [1 1 1]
            %        2         |        [1 1 2]
            %        4         |        [1 2 2]
            antConfig.bsAntSize      = obj.antSize;
            antConfig.ueAntSizes     = 1 + (obj.numLayers.' > [4 2 1]);
            antConfig.antGain        = 25.5;        % Antenna gain (dBi)
            antConfig.noiseFigure    = 6;           % Noise figure (dB)
            antConfig.antTemperature = 290;         % Antenna temperature (K)

        end

        function algParams = get.algParams(obj)
    
            % Algorithmic parameters
            % 'BD': Block diagonalization,
            % 'ZF': Zero forcing,
            % 'RZF': Regularized zero forcing.
            algParams = struct;
            algParams.PrecodingMethod = 'ZF'; % MU-MIMO precoding method: 'BD', 'ZF', 'RZF'
            algParams.ScheduledLayers = 2;
            algParams.PerfectChannelEstimator = true;

        end

    end


end

