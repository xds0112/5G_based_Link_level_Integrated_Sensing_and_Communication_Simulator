%hApplyMultiUserChannels Apply CDL channels for MU-MIMO
%   [CHANNELS,RXDL,RXUL] =
%   hApplyMultiUserChannels(TDDPATTERN,SPECIALSLOT,CARRIER,SCHEDULE,CHANNELS,TXDL,TXUL)
%   applies a multi-user set of CDL channels, returning updated channel
%   configurations, CHANNELS, multi-user set of received downlink
%   waveforms, RXDL, and multi-user set of received uplink waveforms, RXUL.
%   TDDPATTERN is the TDD UL-DL pattern. SPECIALSLOT is the special slot
%   configuration. CARRIER is the carrier configuration. SCHEDULE is a
%   structure specifying the indices of PDSCH and SRS configurations
%   transmitted in this slot. CHANNELS is a multi-user set of CDL channel
%   configurations. TXDL is a multi-user set of transmitted downlink
%   waveforms. TXUL is a multi-user set of transmitted uplink waveforms.
%
%   This function supports applying a channel in the time domain for CDL
%   channels configured with ChannelFiltering = true, and applying a
%   channel in the frequency domain for CDL channels configured with
%   ChannelFiltering = false.

% Copyright 2021 The MathWorks, Inc.

function [channels,rxDL,rxUL] = hApplyMultiUserChannels(tddPattern,specialSlot,carrier,schedule,channels,txDL,txUL)

    % Determine the set of scheduled UEs, i.e. the set of channels active
    % in this slot
    scheduledUEs = unique([schedule.PDSCH schedule.SRS]);

    % Set sample rate and initial time, and calculate maximum delay
    ofdmInfo = txDL.ofdmInfo;
    [channels,maxDelay] = setupChannels(carrier,channels,scheduledUEs,ofdmInfo);

    % Get number of downlink, guard, and uplink samples and symbols
    slotType = tddPattern(mod(carrier.NSlot,numel(tddPattern))+1);

    if (slotType=="D")
        s = {carrier.SymbolsPerSlot 0 0};
    elseif (slotType=="S")
        s = num2cell(specialSlot);
    else % slotType=="U"
        s = {0 0 carrier.SymbolsPerSlot};
    end
    [symD,symG,symU] = deal(s{:});
    [nD,nG,nU] = getSlotSamples(symD,symU,carrier);

    % Remove uplink part of downlink waveform and grid, if required. Note
    % that for uplink slots, this means that the downlink waveform and grid
    % will be empty
    dlWaveform = txDL.dlWaveform;
    dlGrid = txDL.dlGrid;
    dlWaveform(end-nU+1:end,:) = [];
    dlGrid(:,end-symU+1:end,:) = [];

    % Pad waveform to ensure that channel filter is flushed
    nT = size(dlWaveform,2);
    dlWaveform = [dlWaveform; zeros(maxDelay,nT)];

    % For each UE
    rxDL = repmat(newOutputStruct(),1,numel(schedule.PDSCH));
    rxUL = repmat(newOutputStruct(),1,numel(schedule.SRS));
    for ch = scheduledUEs

        % Select channel
        channel = channels(ch).channel;

        % If this UE is scheduled for PDSCH reception
        if (nD~=0 && ismember(ch,schedule.PDSCH))

            % If previously used for uplink, switch back to downlink
            if channel.TransmitAndReceiveSwapped
                channel.swapTransmitAndReceive();
            end

            % Pass downlink waveform through channel
            if (channel.ChannelFiltering)
                s = applyChannel(channels(ch),dlWaveform,ofdmInfo);
            else
                channel.NumTimeSamples = nD + nG + nU;
                s = applyFrequencyDomainChannel(carrier,channels(ch),dlGrid,ofdmInfo);
            end

            % Zero-pad the waveform or grid to account for the uplink part
            % of the slot, if required
            if (channel.ChannelFiltering)
                nR = size(s.rxWaveform,2);
                s.rxWaveform = [s.rxWaveform; zeros(nU,nR)];
            else
                [K,~,nR] = size(s.rxGrid);
                s.rxGrid = [s.rxGrid zeros([K symU nR])];
            end

            % Record the channel output
            rxDL(ch==schedule.PDSCH) = s;

        end

        % If this UE is scheduled for SRS reception
        if (nU~=0 && ismember(ch,schedule.SRS))

            % If not previously used for uplink, switch to uplink
            if ~channel.TransmitAndReceiveSwapped
                channel.swapTransmitAndReceive();
            end

            % Extract uplink waveform and grid for current UE
            ulWaveform = txUL(ch==schedule.SRS).ulWaveform;
            ulGrid = txUL(ch==schedule.SRS).ulGrid;

            % Remove downlink and guard part of uplink waveform and grid,
            % if required, and update start time accordingly
            ulWaveform(1:(nD + nG),:) = [];
            ulGrid(:,1:(symD + symG),:) = [];
            T = channel.InitialTime;
            channel.InitialTime = T + ((nD + nG) / channel.SampleRate);

            % Pad waveform to ensure that channel filter is flushed
            nT = size(ulWaveform,2);
            ulWaveform = [ulWaveform; zeros(maxDelay,nT)]; %#ok<AGROW>

            % Pass uplink waveform through channel
            ofdmInfo = txUL(ch==schedule.SRS).ofdmInfo;
            if (channel.ChannelFiltering)
                s = applyChannel(channels(ch),ulWaveform,ofdmInfo);
            else
                channel.NumTimeSamples = nD + nG + nU;
                s = applyFrequencyDomainChannel(carrier,channels(ch),ulGrid,ofdmInfo);
            end

            % Zero-pad the waveform or grid to account for the downlink and
            % guard part of the slot, if required
            if (channel.ChannelFiltering)
                nR = size(s.rxWaveform,2);
                s.rxWaveform = [zeros(nD + nG,nR); s.rxWaveform];
            else
                [K,~,nR] = size(s.rxGrid);
                s.rxGrid = [zeros([K (symD + symG) nR]) s.rxGrid];
            end

            % Record the channel output
            rxUL(ch==schedule.SRS) = s;

        end

    end

end

% Apply channel to waveform and record output waveform and other
% information in structure 's'
function s = applyChannel(channel,txWaveform,ofdmInfo)

    s = newOutputStruct();
    s.ofdmInfo = ofdmInfo;
    [s.rxWaveform,s.pathGains,s.sampleTimes] = channel.channel(txWaveform);
    s.ChannelFilterDelay = channel.chInfo.ChannelFilterDelay;
    s.pathFilters = channel.pathFilters;

end

% Set sample rate and initial time, and calculate maximum delay
function [channels,maxDelay] = setupChannels(carrier,channels,scheduledUEs,ofdmInfo)

    SR = ofdmInfo.SampleRate;
    slotsPerSubframe = ofdmInfo.SlotsPerSubframe;
    samplesPerSlot = sum(reshape(ofdmInfo.SymbolLengths,[],slotsPerSubframe),1);
    maxDelay = 0;

    for ch = scheduledUEs

        % Select channel
        channel = channels(ch).channel;

        % Set sample rate, update info and store path filters
        if (~isLocked(channel))
            channel.SampleRate = SR;
            if (~channel.ChannelFiltering)
                % At most one sample per OFDM symbol
                L = carrier.SymbolsPerSlot;
                sd = (L * 1e3) / (2 * channel.MaximumDopplerShift);
                channel.SampleDensity = min(channel.SampleDensity,sd);
            end
            channels(ch).chInfo = info(channel);
            channels(ch).pathFilters = getPathFilters(channel);
        end

        % Set initial time
        T = floor(carrier.NSlot / slotsPerSubframe) * 1e-3;
        slotInSubframe = mod(carrier.NSlot,slotsPerSubframe);
        T = T + (sum(samplesPerSlot(1:slotInSubframe)) / SR);
        channel.InitialTime = T;

        % Calculate maximum delay
        chInfo = channels(ch).chInfo;
        delay = ceil(max(chInfo.PathDelays*channel.SampleRate));
        delay = delay + chInfo.ChannelFilterDelay;
        maxDelay = max(maxDelay,delay);

    end

end

function s = newOutputStruct()

    s = struct('rxWaveform',[],'rxGrid',[],'ofdmInfo',[],'pathGains',[],'sampleTimes',[],'noise',[],'ChannelFilterDelay',[],'pathFilters',[]);

end

function s = applyFrequencyDomainChannel(carrier,channel,txGrid,ofdmInfo)

    s = newOutputStruct();
    s.ofdmInfo = ofdmInfo;
    [s.pathGains,s.sampleTimes] = channel.channel();
    offset = channel.chInfo.ChannelFilterDelay;
    s.ChannelFilterDelay = offset;
    s.pathFilters = channel.pathFilters;
    H = nrPerfectChannelEstimate(carrier,s.pathGains,s.pathFilters,offset,s.sampleTimes);
    H = H(:,1:size(txGrid,2),:,:);
    s.rxGrid = applyChannelMatrices(txGrid,H);

end

function out = applyChannelMatrices(in,H)

    [K,L,R,P] = size(H);
    out = zeros([K L R],'like',H);
    a = reshape(in,K*L,P);
    b = permute(H,[1 2 4 3]);
    b = reshape(b,K*L,P,R);
    for r = 1:R
        out(:,:,r) = sum(reshape(a.*b(:,:,r),K,L,P),3);
    end

end

function [nD,nG,nU] = getSlotSamples(symD,symU,carrier)

    ofdmInfo = nrOFDMInfo(carrier);
    L = carrier.SymbolsPerSlot;
    symbolLengths = ofdmInfo.SymbolLengths(mod(carrier.NSlot,carrier.SlotsPerSubframe)*L + (1:L));
    nD = sum(symbolLengths(1:symD));
    nG = sum(symbolLengths(symD+1:end-symU));
    nU = sum(symbolLengths(end-symU+1:end));

end
