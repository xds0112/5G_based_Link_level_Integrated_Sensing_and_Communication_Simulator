%multiUserSelection User selection for MU-MIMO
%   [SCHEDULE,PDSCHs] =
%   multiUserSelection(CSI,TDDPATTERN,SPECIALSLOT,CARRIER,PDSCHs,BSANTSIZE,ALG)
%   performs user selection for MU-MIMO transmission, returning structure
%   SCHEDULE specifying the indices of PDSCH configurations to be
%   transmitted in this slot, and an array of PDSCH configurations, PDSCHs,
%   whose parameters have been updated for MU-MIMO transmissions in this
%   slot. CSI is a multi-user set of channel state information. TDDPATTERN
%   is the TDD UL-DL pattern. SPECIALSLOT is the special slot
%   configuration. CARRIER is the carrier configuration. PDSCHs is a
%   multi-user set of PDSCH configurations. BSANTSIZE is a vector
%   specifying the base station antenna array size. ALG is a structure
%   containing algorithmic options.

% Copyright 2021-2022 The MathWorks, Inc.

function [schedule,PDSCHs] = multiUserSelection(csi,tddPattern,specialSlot,carrier,PDSCHs,bsAntSize,alg)

    % Determine the number of downlink OFDM symbols
    slotType = tddPattern(mod(carrier.NSlot,numel(tddPattern))+1);
    if (slotType=="D")
        nSymbolsDL = carrier.SymbolsPerSlot;
    elseif (slotType=="S")
        nSymbolsDL = specialSlot(1);
    else % slotType=="U"
        nSymbolsDL = 0;
    end

    % If there are no downlink symbols or no CSI has been recorded yet, no
    % PDSCH can be scheduled in this slot
    noCSI = all(isnan(csi(1).nVar(:,:,1)));
    if (nSymbolsDL==0 || noCSI)
        schedule.PDSCH = zeros(1,0);
        return;
    end

    % Determine the number of downlink antennas
    nTxAnts = prod(bsAntSize);

    % Replace NaNs in CSI with nearest non-NaN values, to provide CSI in
    % RBs outside of SRS bandwidth
    numUEs = numel(PDSCHs);
    for ue = 1:numUEs
        nanidx = find(isnan(csi(ue).nVar(:,:,1)));
        nonnanidx = find(~isnan(csi(ue).nVar(:,:,1)));
        replaceidx = arrayfun(@(x)nonnanidx(find(abs(nonnanidx-x)==min(abs(nonnanidx-x)),1)),nanidx);
        csi(ue).nVar(nanidx,:,:) = csi(ue).nVar(replaceidx,:,:);
        csi(ue).H(nanidx,:,:,:) = csi(ue).H(replaceidx,:,:,:);
    end

    % Extract number of layers from the PDSCH configurations and compute
    % total number of layers
    numLayers = [cat(1,PDSCHs.Config).NumLayers];
    totLayers = sum(numLayers);

    % Create variables used to indicate which PRGs are allocated to which
    % UEs, and to store the corresponding precoding matrices; note that
    % the PRG bundle size is assumed to be the same for all users
    prgInfo = communication.dlTransmission.prgInfo(carrier,PDSCHs(1).Extension.PRGBundleSize);
    allUEs = NaN(prgInfo.NPRG,alg.ScheduledLayers);
    allW = NaN(prgInfo.NPRG,alg.ScheduledLayers,nTxAnts);

    % Create vector of served throughput values for each UE and initialize
    % to a very small but non-zero value
    servedThroughput = 1e-6 * ones(1,numUEs);

    % For each PRG
    for PRG = 1:prgInfo.NPRG

        % Use channel and noise estimates from the central RB in the PRG
        RB = floor(mean(find(prgInfo.PRGSet==PRG)));

        % If the PRG is outside the carrier RBs, move to the next PRG
        if (isnan(RB))
            continue;
        end

        % Create channel estimates 'H' and noise estimates 'nVar' across
        % all layers, and create array 'ueLayers' which indicates which
        % layers of the overall estimates correspond to each UE
        H = zeros(totLayers,nTxAnts);
        nVar = zeros(totLayers,1);
        layer = 1;
        ueLayers = cell(1,numUEs);
        for ue = 1:numUEs
            ueLayers{ue} = layer;
            for nu = 1:numLayers(ue)
                H(layer,:) = csi(ue).H(RB + carrier.NStartGrid,1,:,nu);
                nVar(layer) = csi(ue).nVar(RB + carrier.NStartGrid,1,nu);
                layer = layer + 1;
            end
            ueLayers{ue} = ueLayers{ue}:(layer-1);
        end

        % Compute power for each layer in the overall channel
        signalPowers = diag(H*H');

        % Compute total signal power, noise power, and SNR for each UE
        uePowers = cellfun(@sum,mat2cell(signalPowers,numLayers)).';
        ueNoise =  cellfun(@sum,mat2cell(nVar,numLayers)).';
        ueSNR = uePowers ./ ueNoise;

        % Calculate Proportional Fair (PF) metric based on instantaneous
        % single-user capacity and select UE with the highest metric -
        % note that this metric is only used for fairness across the PRGs
        % in this slot, not across slots
        selectedUEs = [];
        selectedLayers = [];
        metricPF = capacity(ueSNR) ./ servedThroughput;
        metricPF(numLayers > alg.ScheduledLayers) = -Inf;
        [maxPF,maxIdx] = max(metricPF);
        if (maxPF > -Inf)
            selectedUEs = [selectedUEs maxIdx]; %#ok<AGROW>
            selectedLayers = [selectedLayers ueLayers{maxIdx}]; %#ok<AGROW>
        end

        % Calculate channel correlation matrix
        scaledH = normalizeMatrix(H);
        R = abs(scaledH*scaledH');

        % While not all UEs have been selected
        while (numel(selectedUEs) < numUEs)

            % Calculate correlation between all UEs and the UEs selected
            % already
            corrLayers = sum(R(selectedLayers,:),1).';
            corrUEs = cellfun(@sum,mat2cell(corrLayers,numLayers)).';

            % Select a further UE subject to the following conditions:
            % The UE has not been selected already
            corrUEs(selectedUEs) = Inf;
            % The number of layers for the UE fits within the target
            % number of scheduled layers
            corrUEs((numel(selectedLayers) + numLayers) > alg.ScheduledLayers) = Inf;
            % The UE has the lowest correlation
            [minCorrelation,minIdx] = min(corrUEs);

            if isfinite(minCorrelation)
                % Select the UE meeting the conditions above
                selectedUEs = [selectedUEs minIdx]; %#ok<AGROW>
                selectedLayers = [selectedLayers ueLayers{minIdx}]; %#ok<AGROW>
            else
                % No UEs met the conditions above, select no further UEs
                break;
            end

        end

        % Extract the channels for each selected UE, 'ueH', and create the
        % overall channel across all selected UEs, 'selectedH'. Also record
        % which selected layer corresponds to which UE, 'ownerUEs'
        numSelectedUEs = numel(selectedUEs);
        selectedNumLayers = numLayers(selectedUEs);
        ownerUEs = zeros(1,sum(selectedNumLayers));
        ueH = cell(1,numSelectedUEs);
        layer = 1;
        for ue = 1:numSelectedUEs
            ueH{ue} = H(ueLayers{selectedUEs(ue)},:).';
            nu = selectedNumLayers(ue);
            ownerUEs(layer:layer+nu-1) = selectedUEs(ue);
            layer = layer + nu;
        end
        selectedH = cat(2,ueH{:}).';

        % Calculate precoding weights using the selected precoding method
        switch alg.PrecodingMethod
            case 'BD'
                if (~isempty(selectedNumLayers))
                    W = blkdiagbfweights(ueH,selectedNumLayers);
                else
                    W = [];
                end
            case {'RZF', 'ZF'}
                layerSNR = signalPowers(selectedLayers) ./ nVar(selectedLayers);
                W = rzf(selectedH,layerSNR,alg);
        end

        % Calculate the capacity per layer and per UE
        SINR = computeSINR(selectedH,W,nVar(selectedLayers));
        layerCapacity = capacity(SINR);
        ueCapacity = cellfun(@sum,mat2cell(layerCapacity,selectedNumLayers)).';

        % Record the served throughput for the selected UEs, on the
        % assumption that the transmission would be successful
        servedThroughput(selectedUEs) = servedThroughput(selectedUEs) + ueCapacity;

        % Record which PRGs are allocated to which UEs, and store the
        % corresponding precoding matrices
        n = size(ownerUEs,2);
        allUEs(PRG,1:n) = ownerUEs;
        allW(PRG,1:n,:) = W;

    end

    % Sort the UEs into vector 'u' which lists them in descending order of
    % the number of overlapping PRGs
    A = zeros(numUEs,numUEs);
    for PRG = 1:prgInfo.NPRG
        thisPRG = allUEs(PRG,:);
        thisPRG(isnan(thisPRG)) = [];
        r = unique(thisPRG);
        A(r,r) = A(r,r) + 1;
    end
    A = triu(A);
    u = [];
    while (numel(u) < numUEs && sum(A(:)) > 0)
        [i,j] = find(A==max(A(:)));
        i = i(1);
        j = j(1);
        u = unique([u i j],'stable');
        A(i,j) = 0;
    end

    % Assign ports and scrambling identities to the UEs in the order 'u'
    % determined above. Orthogonal DM-RS ports are first assigned to the
    % UEs with the most overlapping PRGs, ensuring that the set of DM-RS
    % ports is valid, that is, the set can be signaled by a DCI message
    % (see TS 38.212 Table 7.3.1.2.2-2). When the orthogonal ports have all
    % been used, nonorthogonal DM-RS are configured by reusing the ports
    % and assigning different scrambling identities
    
    c = 0;
    p = [0 1 4 5 2 3 6 7];
    ports = cell(1,numUEs);
    ID = NaN(1,numUEs);

    % For each scheduled UE
    for i = 1:numel(u)

        % Get the number of layers 'nu' for that UE
        ue = u(i);
        nu = numLayers(ue);

        % Update index 'c' to the next position in vector of ports 'p' that
        % gives 'nu' ports that can be signaled by a DCI message, and
        % allocate those ports to the current UE
        c = c + mod(-c,nu);
        ports{ue} = p(mod(c + (0:nu-1),numel(p))+1);

        % When 'c' is greater than or equal to the length of 'p', a set of
        % orthogonal ports is being reused, so assign a dfferent
        % scrambling identity than that assigned to other UEs using those
        % same ports
        ID(ue) = floor(c / numel(p));

        % Update index 'c' to the position beyond the ports used for
        % current UE
        c = c + nu;

    end

    % Establish which UEs are scheduled
    schedule.PDSCH = reshape(unique(allUEs(~isnan(allUEs(:)))),1,[]);

    % For each scheduled UE
    for ue = schedule.PDSCH

        % Configure the symbol allocation according to the TDD
        % configuration
        PDSCHs(ue).Config.SymbolAllocation = [0 nSymbolsDL];

        % Configure the set of PRBs allocated to this UE
        i = find(allUEs==ue);
        [prg,~] = ind2sub(size(allUEs),i);
        ueW = nrExtractResources(i,allW);
        [prg,iprb] = sort(prg);
        prg = unique(prg);
        PDSCHs(ue).Config.PRBSet = find(ismember(prgInfo.PRGSet,prg)) - 1;

        % Configure precoding matrices
        nu = numLayers(ue);
        PDSCHs(ue).Config.NumLayers = nu;
        ueW = permute(reshape(ueW(iprb,:),nu,[],nTxAnts),[1 3 2]);
        PDSCHs(ue).Extension.W = NaN(nu,nTxAnts,prgInfo.NPRG);
        PDSCHs(ue).Extension.W(:,:,prg) = ueW;

        % Configure DM-RS port set
        PDSCHs(ue).Config.DMRS.DMRSPortSet = ports{ue};

        % Configure scrambling identities if required
        if (ID(ue)~=0)
            PDSCHs(ue).Config.DMRS.NIDNSCID = ID(ue);
            PDSCHs(ue).Config.DMRS.NSCID = 1;
        end

    end

end

% Perform ZF or RZF precoding
% For more information see Peel, C.B., et al. "A Vector-Perturbation 
% Technique for Near-Capacity Multiantenna Multiuser Communication—Part I:
% Channel Inversion and Regularization." IEEE Transactions on Signal 
% Processing, Vol. 53, No. 1, February 2005, pp. 195-202
function W = rzf(H,SNR,alg)

    numUEs = size(H,1);
    switch alg.PrecodingMethod
        case 'ZF'
            W = H' / (H*H');
        case 'RZF'
            W = H' / (H*H' + (1 ./ SNR).*eye(numUEs));
    end
    W = normalizeMatrix(W.');

end

% Normalize rows of matrix A such that diagonal elements of A*A' are 1
function A = normalizeMatrix(A)

    A = diag(1 ./ sqrt(diag(A*A'))) * A;

end

% Compute SINR of effective channel H*W, given channel matrix H, precoding
% matrix W and noise variance nVar
function SINR = computeSINR(H,W,nVar)

    W = W.';
    D = H*W;
    S = abs(diag(D)).^2;
    I = real(diag(D*D') - S);
    SINR = S ./ (I + nVar);

end

% Calculate capacity according to Shannon–Hartley theorem. The bandwidth B
% is not considered because it is the same in all contexts where the
% capacity is compared
function C = capacity(SINR)

    C = log2(1 + SINR);

end
