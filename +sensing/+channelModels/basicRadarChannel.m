function rxWaveform = basicRadarChannel(txWaveform, radarEstParams)
% Simulate time-domain multi-target OFDM radar propagation channel.
%
% Author: D.S Xue, Key Laboratory of Universal Wireless Communications,
% Ministry of Education, BUPT.

    %% Communication Parameters
    [txWaveLength, nTxAnts] = size(txWaveform);

    % OFDM parameters
    c      = physconst('Lightspeed');
    fc     = radarEstParams.fc;     % carrier frequency
    lambda = c/fc;                  % wavelength
    fs     = radarEstParams.fs;     % sampling rate/symbol rate
    Ts     = 1/fs;                  % sampling-time interval

    %% Sensing Parameters
    % Range parameters
    pathDelay = 2*radarEstParams.rng.'/c; % echo delay
    symbolShift = ceil(pathDelay./Ts);    % echo symbol shift

    % Velocity parameters
    fd = 2*radarEstParams.vel./lambda; % echo doppler shift

    % Number of targets
    nTargets = numel(radarEstParams.rng);

    %% Multi-Target Radar Propagation Channel
    % Send Tx signal
    sampleTimeTx = (0:Ts:Ts*(txWaveLength-1)).';
    phaseTx      = exp(2j*pi*fc*sampleTimeTx);          % carrier corresponding to Tx, [txWaveLength x nTxAnts]
    txWaveform   = bsxfun(@times, txWaveform, phaseTx); % modulation, [txWaveLength x nTxAnts]

    % Simulate small-scale propagation channel
    largeScaleFading = radarEstParams.lsFading;
    txSteeringVector = radarEstParams.txSteeringVector; % Tx steering vector, [nTxAnts x nTargets]
    rxSteeringVector = radarEstParams.rxSteeringVector; % Rx steering vector, [nRxAnts x nTargets]
    nRxAnts          = size(rxSteeringVector, 1);

    % Parameters initialization
    rxWaveLength = zeros(1, nTargets);
    [rxEchoWave, sampleTimeCh, phaseShift] = deal(cell(1, nTargets));

    for i = 1:nTargets
        % Apply echo path delay and doppler effect
        rxEchoWave{i} = [zeros(symbolShift(i), nTxAnts); txWaveform(1:end-symbolShift(i),:)];

        % Column vector of sampling times
        sampleTimeCh{i} = (0:Ts:Ts*(size(rxEchoWave{i}, 1)-1)).';

        % Apply phase shift caused by Doppler effect
        phaseShift{i} = exp(2j*pi*fd(i)*sampleTimeCh{i});
        rxEchoWave{i} = bsxfun(@times, rxEchoWave{i}, phaseShift{i});

        % Apply large-scale fading
        rxEchoWave{i} = bsxfun(@times, rxEchoWave{i}, largeScaleFading(i));

        % Apply angle steering vector
        rxEchoWave{i} = rxEchoWave{i}*txSteeringVector(:,i)*rxSteeringVector(:,i).';

        % Append zero padding
        rxWaveLength(i) = size(rxEchoWave{i}, 1);
        if rxWaveLength(i) < txWaveLength
            rxEchoWave{i} = [rxEchoWave{i}; zeros(txWaveLength-rxWaveLength(i), nRxAnts)];
        end
    end

    % Combine echoes
    rxWaveform = sum(cat(3, rxEchoWave{:}), 3); % [rxWaveLength x nRxAnts]

    % Apply AWGN
    N0         = sqrt(radarEstParams.N0/2.0);
    noise      = N0*(randn(size(rxWaveform)) + 1j*randn(size(rxWaveform)));
    rxWaveform = rxWaveform + noise;

    % Receive base-band signal
    sampleTimeRx = (0:Ts:Ts*(size(rxWaveform, 1)-1)).';
    phaseRx      = exp(-2j*pi*fc*sampleTimeRx);          % carrier corresponding to Rx [rxWaveLength x 1]
    rxWaveform   = bsxfun(@times, rxWaveform, phaseRx);  % base-band signal [rxWaveLength x nRxAnts]

end
