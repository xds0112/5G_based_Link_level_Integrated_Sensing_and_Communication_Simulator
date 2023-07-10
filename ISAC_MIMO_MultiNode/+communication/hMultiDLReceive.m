%hMultiDLReceive Downlink receiver for MU-MIMO
%   [TBS,CRC,EQSYMBOLS] = hMultiDLReceive(CARRIER,PDSCHs,RX,ALG) performs
%   downlink reception for a multi-user set of received waveforms,
%   returning multi-user sets of transport block sizes, TBS, cyclic
%   redundancy check values, CRC, and equalized symbols EQSYMBOLS. CARRIER
%   is the carrier configuration. PDSCHs is a multi-user set of PDSCH
%   configurations. RX is a multi-user set of received waveforms. ALG is a
%   structure containing algorithmic options.

% Copyright 2021 The MathWorks, Inc.

function [TBS,CRC,eqSymbols] = hMultiDLReceive(carrier,PDSCHs,rx,alg)

    numUEs = numel(rx);
    TBS = cell(1,numUEs);
    CRC = cell(1,numUEs);
    eqSymbols = cell(1,numUEs);

    % For each UE
    for ue = 1:numUEs

        % Extract the configuration for this UE
        pdsch = PDSCHs(ue).Config;
        pdschExt = PDSCHs(ue).Extension;

        % Perform OFDM demodulation (for ChannelFiltering = true)
        rxGrid = rx(ue).rxGrid;
        offset = rx(ue).ChannelFilterDelay;
        if (isempty(rxGrid))
            rxWaveform = rx(ue).rxWaveform;
            rxWaveform = rxWaveform(1+offset:end,:);
            rxGrid = nrOFDMDemodulate(carrier,rxWaveform);
        end

        % Dimensionality information
        K = carrier.NSizeGrid * 12;
        L = carrier.SymbolsPerSlot;
        R = size(rxGrid,3);
        P = pdsch.NumLayers;

        % Perform channel and noise estimation
        if (alg.PerfectChannelEstimator)

            H = nrPerfectChannelEstimate(carrier,rx(ue).pathGains,rx(ue).pathFilters,offset,rx(ue).sampleTimes);
            noiseGrid = nrOFDMDemodulate(carrier,rx(ue).noise(1+offset:end,:));
            nVar = var(noiseGrid(:));

        else

            % Create DM-RS symbols and indices
            dmrsIndices = nrPDSCHDMRSIndices(carrier,pdsch);
            dmrsSymbols = nrPDSCHDMRS(carrier,pdsch);

            % Get subcarrier indices 'k' used by the DM-RS, corresponding 
            % PRG indices 'prg', and set of unique PRGs 'uprg'
            [k,~,~] = ind2sub([K L],dmrsIndices);
            [prg,uprg] = getPRGIndices(carrier,pdschExt,k(:,1));

            % Perform channel estimation for each PRG and layer
            H = zeros([K L R P]);
            prgInfo = hPRGInfo(carrier,pdschExt.PRGBundleSize);
            nVarPRGs = zeros([prgInfo.NPRG P]);
            for i = 1:numel(uprg)

                for p = 1:P
                    [HPRG,nVarPRGs(uprg(i),p)] = nrChannelEstimate(rxGrid,dmrsIndices(prg==uprg(i),p),dmrsSymbols(prg==uprg(i),p),'CDMLengths',[2 2]);
                    H(:,:,:,1:p) = H(:,:,:,1:p) + HPRG;
                end

            end

            % Average noise estimate across PRGs and layers
            nVar = mean(nVarPRGs(uprg,:),'all');

        end

        % Create PDSCH indices and extract allocated PDSCH REs in the
        % received grid and channel estimation
        [pdschIndices,indicesInfo] = nrPDSCHIndices(carrier,pdsch);
        [pdschRx,pdschH,~,pdschHIndices] = nrExtractResources(pdschIndices,rxGrid,H);

        % If perfect channel estimation is configured, the channel
        % estimates must be precoded so that they are w.r.t. layers rather
        % than transmit antennas
        if (alg.PerfectChannelEstimator)
            pdschH = communication.hPRGPrecode(size(H),carrier.NStartGrid,pdschH,pdschHIndices,permute(pdschExt.W,[2 1 3]));
        end

        % Perform equalization
        [eqSymbols{ue},csi] = nrEqualizeMMSE(pdschRx,pdschH,nVar);

        % Perform PDSCH demodulation
        [cws,rxSymbols] = nrPDSCHDecode(carrier,pdsch,eqSymbols{ue},nVar);

        % Apply CSI to demodulated codewords
        csi = nrLayerDemap(csi);
        for c = 1:pdsch.NumCodewords
            Qm = length(cws{c}) / length(rxSymbols{c});
            csi{c} = repmat(csi{c}.',Qm,1);
            cws{c} = cws{c} .* csi{c}(:);
        end

        % Perform DL-SCH decoding
        decodeDLSCH = nrDLSCHDecoder();
        decodeDLSCH.TargetCodeRate = pdschExt.TargetCodeRate;
        decodeDLSCH.LDPCDecodingAlgorithm = 'Normalized min-sum';
        decodeDLSCH.MaximumLDPCIterationCount = 6;
        TBS{ue} = nrTBS(pdsch.Modulation,pdsch.NumLayers,numel(pdsch.PRBSet),indicesInfo.NREPerPRB,pdschExt.TargetCodeRate,pdschExt.XOverhead);
        decodeDLSCH.TransportBlockLength = TBS{ue};
        RV = 0;
        [~,CRC{ue}] = decodeDLSCH(cws,pdsch.Modulation,pdsch.NumLayers,RV);

    end

end

% Calculate PRG indices 'prg', and set of unique PRGs 'uprg' for subcarrier
% indices 'k'
function [prg,uprg] = getPRGIndices(carrier,pdschExt,k)

    prgInfo = hPRGInfo(carrier,pdschExt.PRGBundleSize);
    rb = floor((k-1)/12);
    prg = prgInfo.PRGSet(rb+1);
    uprg = unique(prg).';

end
