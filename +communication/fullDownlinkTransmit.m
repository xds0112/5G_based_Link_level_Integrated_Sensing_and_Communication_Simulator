function rdrTxGrid = fullDownlinkTransmit(numFrames, carrier, pdsch, pdschExt, nTxAnts, newWtx, harq, dlsch)

    % DLSCH encoder object and decoder object    
    encodeDLSCH = dlsch.encodeDLSCH;
    decodeDLSCH = dlsch.decodeDLSCH;

    % Radar Tx and Rx symbols in all slots combined
    rdrTxGrid = [];
    
    % Number of total slots
    nSlots = numFrames*carrier.SlotsPerFrame;

    % Slot loop
    for iSlot = 0:nSlots-1
        % Update the slot index
        carrier.NSlot = iSlot;
    
        % PDSCH generation
        [pdschIndices, pdschInfo] = nrPDSCHIndices(carrier, pdsch);    

        % Calculate transport block sizes
        Xoh_PDSCH = 0;
        trBlkSizes = nrTBS(pdsch.Modulation, pdsch.NumLayers, numel(pdsch.PRBSet), ...
            pdschInfo.NREPerPRB, pdschExt.TargetCodeRate, Xoh_PDSCH);

        % Get new transport blocks and flush decoder soft buffer, as required
        for cwIdx = 1:pdsch.NumCodewords
            if harq.NewData(cwIdx)
                % Create and store a new transport block for transmission
                trBlk = randi([0 1],trBlkSizes(cwIdx),1);
                setTransportBlock(encodeDLSCH,trBlk, cwIdx-1, harq.HARQProcessID);

                % If the previous RV sequence ends without successful
                % decoding, flush the soft buffer
                if harq.SequenceTimeout(cwIdx)
                    resetSoftBuffer(decodeDLSCH, cwIdx-1, harq.HARQProcessID);
                end
            end
        end

        % DL-SCH encoding
        codedTrBlock = encodeDLSCH(pdsch.Modulation, pdsch.NumLayers, pdschInfo.G, ...
            harq.RedundancyVersion, harq.HARQProcessID);

        % Generate PDSCH symbols
        pdschSymbols = nrPDSCH(carrier, pdsch, codedTrBlock);

        % Precode the PDSCH symbols
        precodingWeights = newWtx;
        pdschSymbolsPrecoded = pdschSymbols*precodingWeights;

        % PDSCH DMRS generation
        dmrsSymbols = nrPDSCHDMRS(carrier, pdsch);
        dmrsIndices = nrPDSCHDMRSIndices(carrier, pdsch);

        % PDSCH precoding and mapping
        pdschGrid = nrResourceGrid(carrier, nTxAnts);
        [~, pdschAntIndices] = nrExtractResources(pdschIndices, pdschGrid);
        pdschGrid(pdschAntIndices) = pdschSymbolsPrecoded;

        % PDSCH DMRS precoding and mapping
        for p = 1:size(dmrsSymbols, 2)
            [~, dmrsAntIndices] = nrExtractResources(dmrsIndices(:,p), pdschGrid);
            pdschGrid(dmrsAntIndices) = pdschGrid(dmrsAntIndices) + dmrsSymbols(:,p)*precodingWeights(p,:);
        end

        % Downlink symbols accumulation
        rdrTxGrid = cat(2, rdrTxGrid, pdschGrid);

     end

end