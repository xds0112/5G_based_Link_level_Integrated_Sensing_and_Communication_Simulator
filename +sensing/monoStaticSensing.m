function echoGrid = monoStaticSensing(txWaveform, carrier, waveInfo, bsParams, rdrEstParams, topoParams)
% Simulate gNB-based mono-static sensing.

% Author: D.S.Xue, Key Laboratory of Universal Wireless Communications,
% Ministry of Education, BUPT.
%%
    % Apply large-scale gain
    nTxAnts = prod(bsParams.antConfig.bsAntSize);
    sigAmp  = db2mag(bsParams.txPower-30)*sqrt(waveInfo.Nfft^2/(carrier.NSizeGrid*12*nTxAnts));
    txWaveform = sigAmp*txWaveform;

    % Apply radar propagation channel
    txEcho = sensing.channelModels.basicRadarChannel(txWaveform, waveInfo, bsParams, rdrEstParams, topoParams);

    % OFDM demodulation
    echoGrid = nrOFDMDemodulate(carrier, txEcho);

    % Echo grid dimension validation
    if size(echoGrid,2) < carrier.SymbolsPerSlot
        echoGrid = [echoGrid zeros(size(echoGrid,1), carrier.SymbolsPerSlot-size(echoGrid,2), size(echoGrid,3))];
    end

end
