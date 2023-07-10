%hMultiULReceive Uplink receiver for MU-MIMO
%   [H,NVAR] = hMultiULReceive(CARRIER,SRSs,RX,ALG) performs uplink
%   reception for a multi-user set of received waveforms, returning
%   multi-user set of channel estimates, H, and multi-user set of noise
%   estimates, NVAR. CARRIER is the carrier configuration. SRSs is a
%   multi-user set of SRS configurations. RX is a multi-user set of
%   received waveforms. ALG is a structure containing algorithmic options.

% Copyright 2021 The MathWorks, Inc.

function [H,nVar] = hMultiULReceive(carrier,SRSs,rx,alg)

    numUEs = numel(rx);
    H = cell(1,numUEs);
    nVar = cell(1,numUEs);

    % For each UE
    for ue = 1:numUEs

        % Extract the configuration for this UE
        srs = SRSs(ue);

        % Perform OFDM demodulation (for ChannelFiltering = true)
        rxGrid = rx(ue).rxGrid;
        offset = rx(ue).ChannelFilterDelay;
        if (isempty(rxGrid))
            rxWaveform = rx(ue).rxWaveform;
            rxWaveform = rxWaveform(1+offset:end,:);
            rxGrid = nrOFDMDemodulate(carrier,rxWaveform);
        end

        % Create SRS symbols and indices
        indices = nrSRSIndices(carrier,srs);
        symbols = nrSRS(carrier,srs);

        % Dimensionality information
        K = carrier.NSizeGrid * 12;
        L = carrier.SymbolsPerSlot;
        Nr = size(rxGrid,3);
        Nt = srs.NumSRSPorts;

        % Establish set of resource blocks 'urb' spanned by the SRS, and
        % all subcarrier indices 'k' in those resource blocks
        [k,~,~] = ind2sub([K L],indices);
        rb = floor((k-1)/12);
        urb = unique(rb).';
        k = urb*12 + (1:12).';
        k = k(:);

        % Create channel estimate and noise estimate output
        NCRB = carrier.NStartGrid + carrier.NSizeGrid;
        H{ue} = NaN([NCRB 1 Nr Nt]);
        nVar{ue} = NaN([NCRB 1 Nt]);

        % Perform channel and noise estimation
        nVarRBs = NaN([numel(urb) Nt]);
        if (alg.PerfectChannelEstimator)

            HRBs = nrPerfectChannelEstimate(carrier,rx(ue).pathGains,rx(ue).pathFilters,offset,rx(ue).sampleTimes);
            noiseGrid = nrOFDMDemodulate(carrier,rx(ue).noise(1+offset:end,:));
            nVarRBs(:) = var(noiseGrid(:));

        else

            % Perform channel estimation for each layer
            HRBs = zeros([K L Nr Nt]);
            for p = 1:Nt
                [Hp,nVarRBs(:,p)] = nrChannelEstimate(rxGrid,indices(:,p),symbols(:,p),'CDMLengths',hSRSCDMLengths(srs));
                HRBs(:,:,:,1:p) = HRBs(:,:,:,1:p) + Hp;
            end

        end

        % Average channel estimate across the REs and symbols spanned by
        % the SRS
        HRBs = HRBs(k,:,:,:);
        HRBs = reshape(HRBs,[12 numel(urb) L Nr Nt]);
        HRBs = permute(HRBs,[2 3 4 5 1]);
        HRBs = mean(HRBs,5);
        HRBs = mean(HRBs,2);

        % Assign the averaged channel estimate and noise estimate into the
        % appropriate RBs in the output
        H{ue}(urb + carrier.NStartGrid + 1,:,:,:) = HRBs;
        nVar{ue}(urb + carrier.NStartGrid + 1,:,:) = nVarRBs;

    end

end
