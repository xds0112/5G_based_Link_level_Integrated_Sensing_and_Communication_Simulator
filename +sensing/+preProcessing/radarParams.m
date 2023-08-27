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
    nRxAnts     = nTxAnts;                   % Rx and Tx share the same antenna array
    ele         = topoParams.elevationTgts;  % elevation angle of target (theta),  [1 x nTargets]
    azi         = topoParams.azimuthTgts;    % azimuth angle of target (phi), [1 x nTargets]
    steeringVec = cell(1, nTargets);         % steering vector, [1 x nTargets]
    
    if isa(txArray, 'phased.NRRectangularPanelArray') % UPA model

        spacingX = txArray.Spacing(1);         % array X-axis element spacing
        spacingY = txArray.Spacing(2);         % array Y-axis element spacing
        nAntsX   = txArray.Size(1);            % array X-axis element number
        nAntsY   = txArray.Size(2);            % array Y-axis element number
        antAryX  = (0:1:nAntsX-1)*spacingX;    % array X-axis element indices, [1 x nRxAntsX]
        antAryY  = ((0:1:nAntsY-1)*spacingY)'; % array Y-axis element indices, [nRxAntsY x 1]

        % UPA steering vector, defined in the spheric coordinate system
        aUPA = @(ph, th, m, n)exp(2j*pi*sind(th)*(m*cosd(ph) + n*sind(ph))/lambda);
        
        for t = 1:nTargets
            upaSteeringVec = aUPA(azi(t), ele(t), antAryX, antAryY);
            steeringVec{t} = reshape(upaSteeringVec, nRxAnts, 1);
        end
    
    else  % ULA model

        spacing = txArray.ElementSpacing;     % array element spacing
        antAry  = ((0:1:nRxAnts-1)*spacing)'; % array element, [nRxAnts x 1]

        % ULA steering vector
        aULA = @(ph, m)exp(2j*pi*m*sind(ph)/lambda);

        for t = 1:nTargets
            ulaSteeringVec = aULA(azi(t), antAry);
            steeringVec{t} = ulaSteeringVec;
        end

    end

    steeringVec = cat(2, steeringVec{:}); % [nRxAnts x nTargets]

    radarEstParams.antennaType              = txArray;  % antenna array type
    radarEstParams.azimuthScanScale         = 360;      % azimuth scan scale, normally set to 120°, meaning [-180°, 180°]
    radarEstParams.elevationScanScale       = 180;      % elevation scan scale, normally set to 180°, meaning [-90°, 90°]
    radarEstParams.azimuthScanGranularity   = 1;        % azimuth scan granularity, in degrees
    radarEstParams.elevationScanGranularity = 1;        % elevation scan granularity, in degrees
    radarEstParams.RxSteeringVec            = steeringVec;

    %% Restore Targets' Real Position Configuration
    % CFAR detection zone
    radarEstParams.cfarEstZone = bsParams.cfarEstZone;

    % Sort by SNR in descending order
    radarEstParams.tgtRealPos = struct;
    [~, idx] = sort(radarEstParams.snrdB, 'descend');
    [snrdB(:), r(:), v(:), ele(:), azi(:)] = deal(snrdB(idx), r(idx), v(idx), ele(idx), azi(idx));

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

