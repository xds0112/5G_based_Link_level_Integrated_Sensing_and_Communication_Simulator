%applyMultiUserAWGN Apply AWGN for MU-MIMO
%   OUT = hApplyMultiUserAWGN(CARRIER,IN,SNRdB) applies AWGN to a
%   multi-user set of received waveforms, returning a multi-user set of
%   noisy waveforms, OUT. CARRIER is the carrier configuration. IN is a
%   multi-user set of received waveforms. SNRdB is the signal-to-noise
%   ratio in dB.
%
%   An optional name-value pair CombineWaveforms = false,true provides
%   control over whether or not input signals are combined (i.e. summed)
%   prior to applying AWGN. CombineWaveforms = true is appropriate for base
%   station reception, where all transmitted (UE) waveforms are detected by
%   the base station antennas and then noise is added to those antennas.
%   CombineWaveforms = false is appropriate for UE reception, where noise
%   is separately added to the UE antennas for each UE.
%
%   This function supports applying AWGN in the time domain for channels
%   configured with ChannelFiltering = true, and applying AWGN in the
%   frequency domain for channels configured with ChannelFiltering = false.

% Copyright 2021 The MathWorks, Inc.

function rx = applyMultiUserAWGN(carrier,rx,SNRdB,varargin)

    % Parse the 'CombineWaveforms' option
    persistent ip;
    if (isempty(ip))
        ip = inputParser;
        addParameter(ip,'CombineWaveforms',false);
    end
    parse(ip,varargin{:});
    opts = ip.Results;

    % Combine waveforms if appropriate
    nUEs = numel(rx);
    if (nUEs>0 && opts.CombineWaveforms)
        sumWaveform = rx(1).rxWaveform;
        sumGrid = rx(1).rxGrid;
        for ue = 2:nUEs
            sumWaveform = sumWaveform + rx(ue).rxWaveform;
            sumGrid = sumGrid + rx(ue).rxGrid;
        end
        for ue = 1:nUEs
            rx(ue).rxWaveform = sumWaveform;
            rx(ue).rxGrid = sumGrid;
        end
    end

    % For each UE
    for ue = 1:nUEs

        % Establish dimensionality from either:
        % the received waveform (for ChannelFiltering = true)
        % or
        % the received grid (for ChannelFiltering = false)
        ofdmInfo = rx(ue).ofdmInfo;
        if (~isempty(rx(ue).rxWaveform))
            [T,Nr] = size(rx(ue).rxWaveform);
        else
            L = carrier.SymbolsPerSlot;
            T = sum(ofdmInfo.SymbolLengths(mod(carrier.NSlot,carrier.SlotsPerSubframe)*L + (1:L)));
            T = T + rx(ue).ChannelFilterDelay;
            Nr = size(rx(ue).rxGrid,3);
        end

        % Calculate required noise power spectral density
        SNR = 10^(SNRdB/10);
        N0 = 1 / sqrt(2.0*Nr*ofdmInfo.Nfft*SNR);

        % Create noise, either for the first UE if CombineWaveforms =
        % true, or for every UE if CombineWaveforms = false
        if (ue==1 || ~opts.CombineWaveforms)
            noise = N0 * complex(randn([T Nr]),randn([T Nr]));
            offset = rx(ue).ChannelFilterDelay;
            noiseGrid = nrOFDMDemodulate(carrier,noise(1+offset:end,:));
        end

        % Add noise to either the received waveform or received grid
        % (depending on whether ChannelFiltering is true or false)
        if (~isempty(rx(ue).rxWaveform))
            rx(ue).rxWaveform = rx(ue).rxWaveform + noise;
        else
            rx(ue).rxGrid = rx(ue).rxGrid + noiseGrid;
        end

        % Record noise in the output, this will be used if perfect channel
        % estimation is configured
        rx(ue).noise = noise;

    end

end
