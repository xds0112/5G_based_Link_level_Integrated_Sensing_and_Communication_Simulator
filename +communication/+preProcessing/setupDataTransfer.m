function dataState = setupDataTransfer(carrier,numFrames,numLayers)
% Set up record of PDSCH data transfer
    
    nSlots = carrier.SlotsPerFrame * numFrames;
    TBS = NaN(1,nSlots);
    CRC = NaN(1,nSlots);
    tput = zeros(1,nSlots);
    numUEs = numel(numLayers);
    dataState = repmat(struct('TBS',TBS,'SingleLayerTBS',TBS,'CRC',CRC,'Throughput',tput),1,numUEs);
    
end