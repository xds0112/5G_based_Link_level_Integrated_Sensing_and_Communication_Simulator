function [harq, dlsch, newWtx] = transmissionSetup(pdsch, pdschExt, comChannel, carrier)

    % Set up HARQ
    harq = communication.HARQ(0:15, [0 2 3 1], pdsch.NumCodewords);

    % Create DLSCH encoder object and decoder object
    dlsch.encodeDLSCH = nrDLSCH('MultipleHARQProcesses', true, 'TargetCodeRate', pdschExt.TargetCodeRate);
    dlsch.decodeDLSCH = nrDLSCHDecoder('MultipleHARQProcesses', true, 'TargetCodeRate', pdschExt.TargetCodeRate);

    % Get precoding weight
    estChannelGrid = communication.preProcessing.getInitialChannelEstimate(comChannel, carrier);
    newWtx = communication.preProcessing.getPrecodingMatrix(pdsch.PRBSet, pdsch.NumLayers, estChannelGrid);

end