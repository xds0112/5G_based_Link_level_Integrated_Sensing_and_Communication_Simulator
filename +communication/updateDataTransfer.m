function dataState = updateDataTransfer(dataState,carrier,singleLayerTBS,PDSCH,TBS,CRC)
% Update record of PDSCH data transfer

    nSlot = carrier.NSlot;

    for ue = PDSCH

        TBS_ue = TBS{ue==PDSCH};
        CRC_ue = CRC{ue==PDSCH};
        dataState(ue).SingleLayerTBS(nSlot + 1) = singleLayerTBS;
        dataState(ue).TBS(nSlot + 1) = TBS_ue;
        dataState(ue).CRC(nSlot + 1) = CRC_ue;
        dataState(ue).Throughput(nSlot + 1) = TBS_ue .* (1 - CRC_ue);

    end

end
