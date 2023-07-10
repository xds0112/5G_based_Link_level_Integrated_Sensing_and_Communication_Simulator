%hMultiULTransmit Uplink transmitter for MU-MIMO
%   [TXUL,ACTIVE] = hMultiULTransmit(CARRIER,SRSs) performs uplink
%   transmission for multiple users, returning a multi-user set of uplink
%   waveforms, TXUL, and a vector containing the indices of SRS
%   configurations which are transmitted in this slot, ACTIVE. CARRIER is
%   the carrier configuration. SRSs is a multi-user set of SRS
%   configurations.

% Copyright 2021 The MathWorks, Inc.

function [txUL,active] = hMultiULTransmit(carrier,SRSs)

    % Establish which UEs are active in the current slot, by finding which
    % SRSs have non-empty indices
    indices = arrayfun(@(srs)nrSRSIndices(carrier,srs),SRSs,'UniformOutput',false);
    active = find(cellfun(@(ind)~isempty(ind),indices));
    if (size(active,1)==0)
        active = zeros([1 0]);
    end
    SRSs = SRSs(active);
    indices = indices(active);

    % Create empty output structures
    numUEs = numel(SRSs);
    txUL = repmat(struct('ulWaveform',[],'ulGrid',[],'ofdmInfo',[]),1,numUEs);

    % For each active UE
    for ue = 1:numUEs

        % Extract the configuration for this UE
        srs = SRSs(ue);

        % Create transmit resource grid
        ulGrid = nrResourceGrid(carrier,srs.NumSRSPorts);

        % Create SRS and map to the resource grid
        symbols = nrSRS(carrier,srs);
        ulGrid(indices{ue}) = symbols;

        % Perform OFDM modulation
        txUL(ue).ulGrid = ulGrid;
        [txUL(ue).ulWaveform,txUL(ue).ofdmInfo] = nrOFDMModulate(carrier,ulGrid);

    end

end
