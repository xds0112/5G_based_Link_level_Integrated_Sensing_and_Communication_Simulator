function echoGrid = monoStaticSensing(txWaveform, carrier, waveInfo, bsParams, rdrEstParams, topoParams)
% Simulate gNB-based mono-static sensing.

% Author: D.S.Xue, Key Laboratory of Universal Wireless Communications,
% Ministry of Education, BUPT.
%%
    % Signal amplitude
    nTxAnts = prod(bsParams.antConfig.bsAntSize);
    sigAmp  = db2mag(bsParams.txPower-30)*sqrt(waveInfo.Nfft^2/(carrier.NSizeGrid*12*nTxAnts));
    txWaveform = sigAmp*txWaveform;

    % Apply radar propagation channel
    txEcho = sensing.channelModels.basicRadarChannel(txWaveform, waveInfo, bsParams, rdrEstParams, topoParams);

    % OFDM demodulation
    echoGrid = nrOFDMDemodulate(carrier, txEcho);
end
