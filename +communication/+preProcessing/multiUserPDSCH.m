%multiUserPDSCH PDSCH configurations for MU-MIMO
%   PDSCHs = multiUserPDSCH(NUMLAYERS) creates a multi-user set of PDSCH
%   configurations, PDSCHs. NUMLAYERS is a vector containing the number of
%   layers for each user.

% Copyright 2021 The MathWorks, Inc.

function PDSCHs = multiUserPDSCH(numLayers)

    % Create PDSCH configuration object and set parameters that are common
    % to all UEs
    pdsch = nrPDSCHConfig;
    pdsch.MappingType                 = 'A';
    pdsch.Modulation                  = '64QAM';
    pdsch.DMRS.DMRSConfigurationType  = 1;
    pdsch.DMRS.DMRSLength             = 1;
    pdsch.DMRS.DMRSTypeAPosition      = 2;
    pdsch.DMRS.DMRSAdditionalPosition = 0;

    % Create structure of additional parameters related to precoding and
    % transport block size determination
    extension = struct();

    % Array of precoding matrices, size nu-by-Nt-by-NPRG, where nu is the
    % number of layers, Nt is the number of transmit antennas and NPRG is
    % the number of precoding resource block groups. The array will be
    % initialized by hMultiUserSelection
    extension.W = [];

    % PDSCH PRB bundling (TS 38.214 Section 5.1.2.3)
    % PRGBundleSize is the number of physical RBs (PRB) per precoding
    % resource block group (PRG). Values allowed by the specification are 
    % 2 and 4. The specification also allows a "wideband" mode, where the
    % precoding is the same across the whole bandwidth, which can be
    % configured with PRGBundleSize = []
    extension.PRGBundleSize = 2;

    % Code rate used to calculate transport block sizes
    extension.TargetCodeRate = 772/1024;

    % PDSCH rate matching overhead for TBS (Xoh)
    extension.XOverhead = 0;

    % Create an array of PDSCH configurations, one for each UE, and
    % configure the number of layers and RNTI
    numUEs = numel(numLayers);
    PDSCHs = repmat(struct('Config',pdsch,'Extension',extension),1,numUEs);
    for ue = 1:numUEs

        PDSCHs(ue).Config.NumLayers = numLayers(ue);
        PDSCHs(ue).Config.RNTI = ue;

    end

end
