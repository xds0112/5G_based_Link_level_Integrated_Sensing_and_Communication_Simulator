function rxWaveform = basicRadarChannel(txWaveform, waveInfo, simParams, radarEstParams, topoParams)
% Simulate multi-target OFDM radar propagation channel.

% Author: D.S Xue, Key Laboratory of Universal Wireless Communications,
% Ministry of Education, BUPT.

    %% Communication Parameters
    [txWaveLength,nTxAnts] = size(txWaveform);

    % OFDM parameters
    c      = physconst('Lightspeed');
    fc     = simParams.carrierFrequency;     % carrier frequency
    lambda = c/fc;                           % wave length
    fs     = waveInfo.SampleRate;            % sampling rate/symbol rate
    Ts     = 1/fs;                           % sampling-time interval

    %% Sensing Parameters
    nTargets = numel(simParams.attachedTgts);

    % Range parameters
    pathDelay = 2*topoParams.rangeTgts.'/c;  % echo delay
    % Symbol shift caused by echo path delay
    symbolShift = ceil(pathDelay./Ts);

    % Velocity parameters
    fd = 2*topoParams.velocityTgts./lambda;  % echo doppler shift

    %% Multi-Target Radar Propagation Channel
    % Send Tx signal
    sampleTimeTx = (0:Ts:Ts*(txWaveLength-1)).';        % transpose to make a column vector
    phaseTx      = exp(2j*pi*fc*sampleTimeTx);          % carrier corresponding to Rx [txWaveLength x nTxAnts]
    txWaveform   = bsxfun(@times, txWaveform, phaseTx); % modulation [txWaveLength x nTxAnts]

    % Simulate small-scale propagation channel
    largeScaleFading = radarEstParams.lsFading;
    RxSteeringVector = radarEstParams.RxSteeringVec; % [nTxAnts x nTargets]
    TxSteeringVector = RxSteeringVector; % Tx and Rx shares the same steering vector

    % Parameters initialization
    rxWaveLength = zeros(1, nTargets);
    [rxEchoWave, sampleTimeCh, phaseShift] = deal(cell(1, nTargets));

    for i = 1:nTargets
        % Apply echo path delay and doppler effect
        rxEchoWave{i}   = [zeros(symbolShift(i), nTxAnts); txWaveform(1:end-symbolShift(i),:)];
        sampleTimeCh{i} = (0:Ts:Ts*(size(rxEchoWave{i}, 1)-1)).';       % column vector of sampling times
        phaseShift{i}   = exp(2j*pi*fd(i)*sampleTimeCh{i});             % phase shift caused by Doppler effect
        rxEchoWave{i}   = bsxfun(@times, rxEchoWave{i}, phaseShift{i}); % apply phase shift

        % Apply large-scale fading
        rxEchoWave{i} = bsxfun(@times, rxEchoWave{i}, largeScaleFading(i));

        % Apply angle steering vector
        rxEchoWave{i} = rxEchoWave{i}*RxSteeringVector(:,i)*TxSteeringVector(:,i).';

        % Append zero padding
        rxWaveLength(i) = size(rxEchoWave{i}, 1);
        if rxWaveLength(i) < txWaveLength
            rxEchoWave{i} = [rxEchoWave{i}; zeros(txWaveLength-rxWaveLength(i), nTxAnts)];
        end
    end

    % Combine echoes
    rxWaveform = sum(cat(3,rxEchoWave{:}),3); % [rxWaveLength x nRxAnts]

    % Apply AWGN
    N0         = sqrt(radarEstParams.N0/2.0);
    noise      = N0*(randn(size(rxWaveform)) + 1j*randn(size(rxWaveform)));
    rxWaveform = rxWaveform + noise;

    % Receive base-band signal
    sampleTimeRx = (0:Ts:Ts*(size(rxWaveform, 1)-1)).';  % transpose to make a column vector
    phaseRx      = exp(-2j*pi*fc*sampleTimeRx);          % carrier corresponding to Rx [rxWaveLength x 1]
    rxWaveform   = bsxfun(@times, rxWaveform, phaseRx);  % base-band signal [rxWaveLength x nRxAnts]

end
