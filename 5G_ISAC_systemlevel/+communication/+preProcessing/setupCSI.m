function csi = setupCSI(carrier,bsAntSize,ueAntSizes)
% Set up record of CSI obtained via SRS
    
    NCRB = carrier.NSizeGrid + carrier.NStartGrid;
    numUEs = size(ueAntSizes,1);
    csi = repmat(struct('H',[],'nVar',[],'NSlot',[]),1,numUEs);
    P = prod(bsAntSize);
    
    for ue = 1:numUEs
        
        R = prod(ueAntSizes(ue,:));
        csi(ue).H = NaN([NCRB 1 P R]);
        csi(ue).nVar = NaN([NCRB 1 R]);
        csi(ue).NSlot = NaN([NCRB 1]);
        
    end
    
end

