function radarEstParams = radarParams(nSlots, carrier, waveInfo, bsParams, topoParams)
% Calculate Radar SNR point and estimation resolutions.

% Author: D.S Xue, Key Laboratory of Universal Wireless Communications,
% Ministry of Education, BUPT.

    %% Radar Estimation Parameters
    radarEstParams = struct(); 

    %% Radar SNR Point Calculation  
    tgtParams = bsParams.attachedTgts;
    nTargets  = numel(tgtParams);
    nSc       = carrier.NSizeGrid*12;                % Subcarriers
    nSym      = nSlots*waveInfo.SymbolsPerSlot;      % OFDM symbols
    uf        = 1;                                   % Freq domain occupation spacing factor
    ut        = 1;                                   % Time domain occupation spacing factor  
    nTxAnts   = prod(bsParams.antConfig.bsAntSize);  % Base station antenna size

    % OFDM Parameters
    c      = physconst('Lightspeed');
    fc     = bsParams.carrierFrequency;      % Carrier frequency
    scs    = carrier.SubcarrierSpacing*1e3;  % Subcarrier spacing in Hz
    lambda = c/fc;                           % Wavelength
    fs     = waveInfo.SampleRate;            % Sample rate
    Ts     = 1/fs;                           % Sampling-time interval
    Tofdm  = 1/scs;                          % Duration of an effective OFDM symbol
    if strcmp(carrier.CyclicPrefix,'normal')
        Tcp = Ts*ceil(nSc/8);                % Duration of normal OFDM CP
    else % 'extended'
        Tcp = Ts*ceil(nSc/4);                % Duration of extended OFDM CP
    end
    Tsri = Tofdm + Tcp;                      % Duration of a whole OFDM symbol

    % Physical parameters
    kBoltz = physconst('Boltzmann');
    NF     = db2pow(bsParams.antConfig.noiseFigure);
    Teq    = bsParams.antConfig.antTemperature + 290*(NF-1); % Equivalent noise temperature (in K)
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
    radarEstParams.fc       = fc;           % Central frequency
    radarEstParams.fs       = fs;           % Sample rate
    radarEstParams.Tsri     = Tsri;         % Duration of a whole OFDM symbol
    radarEstParams.N0       = N0;           % Noise
    radarEstParams.lsFading = sqrt(Pr./Pt); % Large-scale fading
    radarEstParams.snrdB    = snrdB;        % SNR points (dB) [1 x nTargets]
    radarEstParams.Pfa      = bsParams.Pfa;
    
    %% Resolutions, Performance Limits and Antenna Array Steering Vector

    % Range Parameters
    nIFFT                = 2^nextpow2(nSc/uf);
    radarEstParams.nIFFT = nIFFT;                % 2^n closest to subcarrier numbers
    radarEstParams.rRes  = c/(2*(scs*uf)*nIFFT); % Range resolution
    radarEstParams.rMax  = c/(2*(scs*uf));       % Maxium unambiguous range

    % Velocity Parameters
    nFFT                = 2^nextpow2(nSym/ut);
    radarEstParams.nFFT = nFFT;                      % 2^n closest to symbol numbers
    radarEstParams.vRes = lambda/(2*(Tsri*ut)*nFFT); % Velocity resolution
    radarEstParams.vMax = lambda/(2*(Tsri*ut));      % Maxium unambiguous velocity

    % Antenna array orientation parameters
    txArray     = bsParams.txArray;
    ele         = topoParams.elevationTgts;  % Elevation angle of target,  [1 x nTargets]
    azi         = topoParams.azimuthTgts;    % Azimuth angle of target, [1 x nTargets]
    steeringVec = cell(1,nTargets);      % Steering vector, [1 x nTargets]
    
    if isa(txArray,'phased.NRRectangularPanelArray')    % UPA model

        disp('UPA model is not supported for radar sensing services')
        return
    
    else                                                % ULA model
        rxArySpacing = txArray.ElementSpacing;          % Antenna array element spacing
        nRxAnts      = nTxAnts;                         % Rx and Tx share the antenna array
        rxAntAry     = ((0:1:nRxAnts-1)*rxArySpacing)'; % Antenna array element, [nRxAnts x 1]
        aryDelay     = zeros(nRxAnts,nTargets);         % Antenna array delay, [nRxAnts x nTargets]

        for t = 1:nTargets
            aryDelay(:,t)  = rxAntAry.*sind(azi(t))/c;    % Antenna array delay, [nRxAnts x 1]
            steeringVec{t} = exp(2j*pi*fc*aryDelay(:,t)); % Array azimuth steering vector, [nRxAnts x 1]
        end

    end

    steeringVec = cat(2,steeringVec{:}); % [nRxAnts x nTargets]
    radarEstParams.RxSteeringVec = steeringVec;

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

