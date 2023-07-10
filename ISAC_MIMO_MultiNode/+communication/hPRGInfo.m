%hPRGInfo Precoding Resource Block Group (PRG) related information
%   INFO = hPRGInfo(CARRIER,BUNDLESIZE) provides information related to
%   Physical Resource Block GROUP (PRG) bundling, defined in TS 38.214
%   Section 5.1.2.3.
%
%   CARRIER is a carrier configuration object, <a
%   href="matlab:help('nrCarrierConfig')"
%   >nrCarrierConfig</a>. Only these
%   object properties are relevant for this function:
%   NSizeGrid         - Number of resource blocks in carrier resource grid
%                       (1...275)
%   NStartGrid        - Start of carrier resource grid relative to CRB 0
%                       (0...2199)
%
%   BUNDLESIZE is the PRG bundle size (2, 4, or [] to signify 'wideband').
%
%   INFO is a structure containing the fields:
%   NPRG     - Number of PRGs in carrier resource blocks 0...NCRB-1 where
%              NCRB-1 is the CRB index of the last resource block in the
%              carrier grid (i.e. NCRB = NStartGrid + NSizeGrid)
%   PRGSet   - 1-based PRG indices for each RB in the carrier grid
%
%   See also hPRGPrecode.

% Copyright 2021 The MathWorks, Inc.

function info = hPRGInfo(carrier,bundleSize)

    % Calculate the number of carrier resource blocks (CRB) spanning the
    % carrier grid including the starting CRB offset
    NCRB = carrier.NStartGrid + carrier.NSizeGrid;

    % Handle the case of empty bundleSize, which configures a single
    % fullband PRG
    if (isempty(bundleSize))
        Pd_BWP = NCRB;
    else
        Pd_BWP = bundleSize;
    end

    % Calculate the number of precoding resource block groups
    NPRG = ceil(NCRB / Pd_BWP);

    % Calculate the 1-based PRG indices for each RB in the carrier grid
    prgset = repmat(1:NPRG,[Pd_BWP 1]);
    prgset = reshape(prgset(carrier.NStartGrid + (1:carrier.NSizeGrid).'),[],1);

    % Create the info output
    info.NPRG = NPRG;
    info.PRGSet = prgset;

end
