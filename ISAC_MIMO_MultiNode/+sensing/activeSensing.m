function echoGrid = activeSensing(txGrid,carrier,waveInfo,gNBParams,rdrEstParams)
% Simulate gNB-based (mono-static) active sensing.

% Author: D.S.Xue, Key Laboratory of Universal Wireless Communications,
% Ministry of Education, BUPT.
%%
    % OFDM modulation
    txWaveform = nrOFDMModulate(carrier,txGrid);

    % Calculate the amplitude of the transmitted signal
    sigAmp = db2mag(gNBParams.TxPower-30)*sqrt(waveInfo.Nfft^2/(carrier.NSizeGrid*12*gNBParams.NTxAnts));
    txWaveform = sigAmp*txWaveform;

    % Apply radar propagation channel
    txEcho = sensing.hRadarChannel(txWaveform,waveInfo,gNBParams,rdrEstParams);

    % OFDM demodulation
    echoGrid = nrOFDMDemodulate(carrier,txEcho);

    % Echo grid dimension validation
    if size(echoGrid,2) < carrier.SymbolsPerSlot
        echoGrid = [echoGrid zeros(size(echoGrid,1),carrier.SymbolsPerSlot-size(echoGrid,2),size(echoGrid,3))];
    end

end
