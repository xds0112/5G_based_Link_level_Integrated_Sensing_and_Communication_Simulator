%hMultiUserSRS SRS configurations for MU-MIMO
%   SRSs = hMultiUserSRS(TDDPATTERN,SPECIALSLOT,CARRIER,NUMLAYERS) creates
%   a multi-user set of SRS configurations, SRSs. TDDPATTERN is the TDD
%   UL-DL pattern. SPECIALSLOT is the special slot configuration. CARRIER
%   is the carrier configuration. NUMLAYERS is a vector containing the
%   number of layers for each user.

% Copyright 2021 The MathWorks, Inc.

function SRSs = hMultiUserSRS(tddPattern,specialSlot,carrier,numLayers)

    % Create SRS configuration object and set parameters that are common to
    % all UEs
    srs = nrSRSConfig;
    srs.ResourceType = 'periodic';
    srs.Repetition = 1;    
    srs.BSRS = 0;
    srs.BHop = 1;
    srs.FrequencyStart = 0;
    srs.NRRC = 0;
    srs.NumSRSSymbols = 1;

    % Configure SRS for widest bandwidth that will fit in the carrier
    CSRS = srs.BandwidthConfigurationTable{:,"C_SRS"};
    mSRS0 = srs.BandwidthConfigurationTable{:,"m_SRS_0"};
    srs.CSRS = CSRS(find(mSRS0 <= carrier.NSizeGrid,1,'last'));

    % Schedule SRS in the first special slot of the period, or the first
    % uplink slot if there are no uplink symbols in the special slot    
    slotPeriod = numel(tddPattern);
    for slotType = ["S" "U"]

        slotOffset = find(tddPattern==slotType,1,'first') - 1;
        srs.SRSPeriod = [slotPeriod slotOffset];
        if (slotType=="S")
            s = num2cell(specialSlot);
        else % slotType=="U"
            s = {0 0 carrier.SymbolsPerSlot};
        end
        [symD,symG,symU] = deal(s{:});

        if (symU~=0)
            break;
        end

    end

    % Find the lowest KTC value sufficient to configure SRSs for all UEs
    KTCs = [2 4];
    numUEs = numel(numLayers);
    KTC = numUEs / symU;
    if (KTC > max(KTCs))
        error('Insufficient KBarTC and SymbolStart positions for the configured number of UEs.');
    else
        srs.KTC = KTCs(find(KTCs >= KTC,1,'first'));
    end

    % Create an array of SRS configurations, one for each UE, and
    % configure the number of ports, OFDM symbol number and comb offset
    SRSs = repmat(srs,1,numUEs);
    for ue = 1:numUEs

        SRSs(ue).NumSRSPorts = numLayers(ue);
        SRSs(ue).SymbolStart = symD + symG + mod(ue-1,symU);
        SRSs(ue).KBarTC = floor((ue-1) / symU);

    end

end
