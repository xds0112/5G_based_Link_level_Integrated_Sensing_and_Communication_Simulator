function echoGrid = monoStaticSensing(txWaveform, carrier, waveInfo, bsParams, rdrEstParams)
% Simulate gNB-based mono-static sensing.

% Author: D.S.Xue, Key Laboratory of Universal Wireless Communications,
% Ministry of Education, BUPT.
%%
    % Signal amplitude
    nTxAnts = prod(bsParams.antConfig.bsAntSize);
    sigAmp  = db2mag(bsParams.txPower-30)*sqrt(waveInfo.Nfft^2/(carrier.NSizeGrid*12*nTxAnts));
    txWaveform = sigAmp*txWaveform;

    % Pass txWaveform through radar propagation channel
    txEcho = sensing.channelModels.basicRadarChannel(txWaveform, rdrEstParams);

    % OFDM demodulation
    echoGrid = nrOFDMDemodulate(carrier, txEcho);

end
