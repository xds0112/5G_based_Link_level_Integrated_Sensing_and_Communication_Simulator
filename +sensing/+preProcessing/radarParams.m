function radarEstParams = radarParams(nSlots, carrier, waveInfo, bsParams, topoParams)
% Calculate Radar SNR point and estimation resolutions.

% Author: D.S Xue, Key Laboratory of Universal Wireless Communications,
% Ministry of Education, BUPT.

    %% Radar Estimation Parameters
    radarEstParams = struct(); 

    %% Radar SNR Point Calculation  
    tgtParams = bsParams.attachedTgts;
    nTargets  = numel(tgtParams);
    nSc       = carrier.NSizeGrid*12;                % subcarriers
    nSym      = nSlots*waveInfo.SymbolsPerSlot;      % OFDM symbols
    uf        = 1;                                   % freq domain occupation spacing factor
    ut        = 1;                                   % time domain occupation spacing factor  
    nTxAnts   = prod(bsParams.antConfig.bsAntSize);  % base station antenna size

    % OFDM Parameters
    c      = physconst('Lightspeed');
    fc     = bsParams.carrierFrequency;      % carrier frequency
    scs    = carrier.SubcarrierSpacing*1e3;  % subcarrier spacing in Hz
    lambda = c/fc;                           % wavelength
    fs     = waveInfo.SampleRate;            % sample rate
    Ts     = 1/fs;                           % sampling-time interval
    Tofdm  = 1/scs;                          % duration of an effective OFDM symbol
    if strcmp(carrier.CyclicPrefix,'normal')
        Tcp = Ts*ceil(nSc/8);                % duration of normal OFDM CP
    else % 'extended'
        Tcp = Ts*ceil(nSc/4);                % duration of extended OFDM CP
    end
    Tsri = Tofdm + Tcp;                      % duration of a whole OFDM symbol

    % Physical parameters
    kBoltz = physconst('Boltzmann');
    NF     = db2pow(bsParams.antConfig.noiseFigure);
    Teq    = bsParams.antConfig.antTemperature + 290*(NF-1); % equivalent noise temperature (in K)
    N0     = fs*kBoltz*Teq;
    Pt     = db2pow(bsParams.txPower-30)*sqrt(waveInfo.Nfft^2/(carrier.NSizeGrid*12*nTxAnts));  % in W
    Ar     = db2pow(bsParams.antConfig.antGain);
    At     = Ar;
    
    % Large-Scale Fading
    rcs = zeros(1,nTargets);
    for itg = 1:nTargets
        rcs(itg) = tgtParams(itg).rcs; 
    end
    r     = topoParams.rangeTgts;
    v     = topoParams.velocityTgts;
    Pr    = Pt.*At.*Ar.*(lambda^2*rcs)./((4*pi)^3*r.^4); % Rx power
    snr   = Pr./N0;
    snrdB = pow2db(snr);
    
    % radarEstParams
    radarEstParams.fc       = fc;           % central frequency
    radarEstParams.fs       = fs;           % sample rate
    radarEstParams.Tsri     = Tsri;         % duration of a whole OFDM symbol
    radarEstParams.N0       = N0;           % noise
    radarEstParams.lsFading = sqrt(Pr./Pt); % large-scale fading
    radarEstParams.snrdB    = snrdB;        % SNR points (dB) [1 x nTargets]
    radarEstParams.Pfa      = bsParams.Pfa;
    
    %% Resolutions, Performance Limits and Antenna Array Steering Vector

    % Range Parameters
    nIFFT                = 2^nextpow2(nSc/uf);
    radarEstParams.nIFFT = nIFFT;                % 2^n closest to subcarrier numbers
    radarEstParams.rRes  = c/(2*(scs*uf)*nIFFT); % range resolution
    radarEstParams.rMax  = c/(2*(scs*uf));       % maxium unambiguous range

    % Velocity Parameters
    nFFT                = 2^nextpow2(nSym/ut);
    radarEstParams.nFFT = nFFT;                      % 2^n closest to symbol numbers
    radarEstParams.vRes = lambda/(2*(Tsri*ut)*nFFT); % velocity resolution
    radarEstParams.vMax = lambda/(2*(Tsri*ut));      % maxium unambiguous velocity

    % Antenna array orientation parameters
    txArray     = bsParams.txArray;
    ele         = topoParams.elevationTgts;  % elevation angle of target,  [1 x nTargets]
    azi         = topoParams.azimuthTgts;    % azimuth angle of target, [1 x nTargets]
    steeringVec = cell(1, nTargets);         % steering vector, [1 x nTargets]
    nRxAnts     = nTxAnts;                   % Rx and Tx share the same antenna array
    
    if isa(txArray, 'phased.NRRectangularPanelArray') % UPA model

        rxArySpacingX = txArray.Spacing(1);                % array X-axis element spacing
        rxArySpacingY = txArray.Spacing(2);                % array Y-axis element spacing
        nRxAntsX      = txArray.Size(1);                   % array X-axis element number
        nRxAntsY      = txArray.Size(2);                   % array Y-axis element number
        rxAntAryX     = ((0:1:nRxAntsX-1)*rxArySpacingX)'; % array X-axis element indices, [nRxAntsX x 1]
        rxAntAryY     = ((0:1:nRxAntsY-1)*rxArySpacingY)'; % array Y-axis element indices, [nRxAntsY x 1]
        aryDelayX     = zeros(nRxAntsX, nTargets);         % array X-axis time delay, [nRxAntsX x nTargets]
        aryDelayY     = zeros(nRxAntsY, nTargets);         % array Y-axis time delay, [nRxAntsY x nTargets]
        
        % UPA steering vector is defined in the spheric coordinate system
        for t = 1:nTargets
            aryDelayX(:,t) = rxAntAryX.*sind(azi(t)).*sind(ele(t))./c;
            aryDelayY(:,t) = rxAntAryY.*cosd(azi(t)).*sind(ele(t))./c;
            steeringVecX   = exp(2j.*pi.*fc.*aryDelayX(:,t));            % array X-axis steering vector, [nRxAntsX x 1]
            steeringVecY   = exp(2j.*pi.*fc.*aryDelayY(:,t));            % array Y-axis steering vector, [nRxAntsY x 1]
            steeringMat    = kron(steeringVecX, steeringVecY);           % array steering matrix, [nRxAntsX x nRxAntsY]
            steeringVec{t} = reshape(steeringMat, nRxAntsX*nRxAntsY, 1); % array steering vector, [nRxAnts x 1]
        end
    
    else  % ULA model

        rxArySpacing = txArray.ElementSpacing;          % array element spacing
        rxAntAry     = ((0:1:nRxAnts-1)*rxArySpacing)'; % array element, [nRxAnts x 1]
        aryDelay     = zeros(nRxAnts, nTargets);        % array delay, [nRxAnts x nTargets]

        for t = 1:nTargets
            aryDelay(:,t)  = rxAntAry.*sind(azi(t))./c;
            steeringVec{t} = exp(2j.*pi.*fc.*aryDelay(:,t)); % array steering vector, [nRxAnts x 1]
        end

    end

    steeringVec = cat(2,steeringVec{:}); % [nRxAnts x nTargets]

    radarEstParams.scanScale       = 120; % scan scale, normally set to 120°, meaning [-60°, 60°]
    radarEstParams.scanGranularity = .5;  % scan granularity, in degrees
    radarEstParams.RxSteeringVec   = steeringVec;

    %% Restore Targets' Real Position Configuration
    % CFAR detection zone
    radarEstParams.cfarEstZone = bsParams.cfarEstZone;

    % Sort by SNR in descending order
    radarEstParams.tgtRealPos = struct;
    [~,idx] = sort(radarEstParams.snrdB,'descend');
    [snrdB(:),r(:),v(:),ele(:),azi(:)] = deal(snrdB(idx),r(idx),v(idx),ele(idx),azi(idx));

    % Assignment
    for i = 1:nTargets

        radarEstParams.tgtRealPos(i).ID        = i;
        radarEstParams.tgtRealPos(i).Range     = r(i);
        radarEstParams.tgtRealPos(i).Velocity  = v(i);
        radarEstParams.tgtRealPos(i).Elevation = ele(i);
        radarEstParams.tgtRealPos(i).Azimuth   = azi(i);
        radarEstParams.tgtRealPos(i).snrdB     = snrdB(i);

    end

end

