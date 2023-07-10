function csi = updateCSI(csi,carrier,SRS,H,nVar)
% Update record of CSI obtained via SRS
    
    for ue = SRS
        
        H_ue = H{ue==SRS};
        nVar_ue = nVar{ue==SRS};        
        idx = find(all(~isnan(nVar_ue),3));
        csi(ue).H(idx,:,:,:) = H_ue(idx,:,:,:);
        csi(ue).nVar(idx,:,:) = nVar_ue(idx,:,:);
        csi(ue).NSlot(idx) = carrier.NSlot;
        
    end
    
end