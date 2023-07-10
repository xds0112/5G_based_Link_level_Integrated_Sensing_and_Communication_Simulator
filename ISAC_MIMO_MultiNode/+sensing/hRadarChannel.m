function rxWaveform = hRadarChannel(txWaveform, waveInfo, simParams, radarEstParams)
% Simulate Multi-Target OFDM Radar Propagation Channel.

% Author: D.S Xue, Key Laboratory of Universal Wireless Communications,
% Ministry of Education, BUPT.

    %% Communication Parameters
    [txWaveLength,nTxAnts] = size(txWaveform);

    % OFDM parameters
    c      = physconst('Lightspeed');
    fc     = simParams.CarrierFrequency;     % Carrier frequency
    lambda = c/fc;                           % Wave length
    fs     = waveInfo.SampleRate;            % Sampling rate/symbol rate
    Ts     = 1/fs;                           % Sampling-time interval

    %% Sensing Parameters
    nTargets = simParams.numTargets;

    % Range parameters
    r         = simParams.Range.'; % Transpose to make a column vector
    pathDelay = 2*r/c;             % Echo delay

    % Velocity parameters
    v  = simParams.Vel.';     % Transpose to make a column vector
    fd = 2*v/lambda;          % Echo doppler shift

    %% Multi-Target Radar Propagation Channel
    % Send Tx signal
    sampleTimeTx = (0:Ts:Ts*(txWaveLength-1)).';      % Transpose to make a column vector
    phaseTx      = exp(2j*pi*fc*sampleTimeTx);        % Carrier corresponding to Rx [txWaveLength x nTxAnts]
    txWaveform   = bsxfun(@times,txWaveform,phaseTx); % Modulation [txWaveLength x nTxAnts]

    % Simulate small-scale propagation channel
    largeScaleFading = radarEstParams.lsFading;
    RxSteeringVector = radarEstParams.RxSteeringVec; % [nTxAnts x nTargets]
    TxSteeringVector = RxSteeringVector; % Tx and Rx steering vectors are the same in the sensing channel model
    rxWaveLength = zeros(1,nTargets);
    rxEchoWave   = cell(1,nTargets);
    sampleTimeCh = cell(1,nTargets); 
    phaseShift   = cell(1,nTargets);
    symbolShift  = ceil(pathDelay./Ts); % Symbol shift caused by echo path delay [1 x nTargets]

    for i = 1:nTargets
        % Apply echo path delay and doppler effect
        rxEchoWave{i}   = [zeros(symbolShift(i),nTxAnts); txWaveform(1:end-symbolShift(i),:)];
        sampleTimeCh{i} = (0:Ts:Ts*(size(rxEchoWave{i},1)-1)).';      % Column vector of sampling times
        phaseShift{i}   = exp(2j*pi*fd(i)*sampleTimeCh{i});           % Phase shift caused by doppler effect
        rxEchoWave{i}   = bsxfun(@times,rxEchoWave{i},phaseShift{i}); % Apply phase shift

        % Apply large-scale fading
        rxEchoWave{i} = bsxfun(@times,rxEchoWave{i},largeScaleFading(i));

        % Apply angle steering vector
        rxEchoWave{i} = rxEchoWave{i}*RxSteeringVector(:,i)*TxSteeringVector(:,i).';

        % Append zero padding
        rxWaveLength(i) = size(rxEchoWave{i},1);
        if rxWaveLength(i) < txWaveLength
            rxEchoWave{i} = [rxEchoWave{i}; zeros(txWaveLength-rxWaveLength(i),nTxAnts)];
        end
    end

    % Combine echoes
    rxWaveform = sum(cat(3,rxEchoWave{:}),3);      % [rxWaveLength x nRxAnts]

    % Apply AWGN
    N0         = sqrt(radarEstParams.N0/2.0);
    noise      = N0*(randn(size(rxWaveform)) + 1j*randn(size(rxWaveform)));
    rxWaveform = rxWaveform + noise;

    % Receive base-band signal
    sampleTimeRx = (0:Ts:Ts*(size(rxWaveform,1)-1)).'; % Transpose to make a column vector
    phaseRx      = exp(-2j*pi*fc*sampleTimeRx);        % Carrier corresponding to Rx [rxWaveLength x 1]
    rxWaveform   = bsxfun(@times,rxWaveform,phaseRx);  % Base-band signal [rxWaveLength x nRxAnts]

end