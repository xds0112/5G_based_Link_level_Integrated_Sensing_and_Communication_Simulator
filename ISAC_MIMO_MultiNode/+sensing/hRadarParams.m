function radarEstParams = hRadarParams(carrier, waveInfo, simParams)
% Calculate Radar SNR point and estimation resolutions.

% Author: D.S Xue, Key Laboratory of Universal Wireless Communications,
% Ministry of Education, BUPT.

    %% Radar Estimation Parameters
    radarEstParams = struct(); 

    %% Radar SNR Point Calculation  
    nTargets = simParams.numTargets;
    nSlots   = simParams.NFrames*carrier.SlotsPerFrame;
    nSc      = carrier.NSizeGrid*12;               % Subcarriers
    nSym     = nSlots*waveInfo.SymbolsPerSlot;     % OFDM symbols
    uf       = 1;                                  % Freq domain occupation spacing factor
    ut       = 1;                                  % Time domain occupation spacing factor
    antFFT   = 256;                                % FFT points for angle estimation (only in 3D-FFT)   

    % OFDM Parameters
    c      = physconst('Lightspeed');
    fc     = simParams.CarrierFrequency;     % Carrier frequency
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
    NF     = db2pow(simParams.RxNoiseFigure);
    Teq    = simParams.RxAntTemperature + 290*(NF-1); % Equivalent noise temperature (in K)
    N0     = fs*kBoltz*Teq;
    Pt     = db2pow(simParams.TxPower-30)*sqrt(waveInfo.Nfft^2/(carrier.NSizeGrid*12*simParams.NTxAnts));  % in W
    Ar     = db2pow(simParams.AntGain);
    At     = Ar;
    
    % Large-Scale Fading
    rcs   = simParams.RxRCS; 
    r     = simParams.Range;
    v     = simParams.Vel;
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
    radarEstParams.Pfa      = simParams.Pfa;
    
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
    txArray     = simParams.txArray;
    ele         = simParams.MoveDir(2,:);  % Elevation angle of target  [1 x nTargets]
    azi         = simParams.MoveDir(1,:);  % Azimuth angle of target [1 x nTargets]
    steeringVec = cell(1,nTargets);        % Steering/response vector [1 x nTargets]
    
    if isa(txArray,'phased.NRRectangularPanelArray')      % UPA

        disp('UPA model is not supported for radar sensing')
        return
    
    else % ULA
        rxArySpacing = txArray.ElementSpacing;          % Antenna array element spacing
        nRxAnts      = simParams.NTxAnts;
        rxAntAry     = ((0:1:nRxAnts-1)*rxArySpacing)'; % Antenna Array Element [nRxAnts x 1]
        eleRes       = [];
        aziRes       = lambda/(rxArySpacing*antFFT);    % Azimuth Resolution
        aryDelay     = zeros(nRxAnts,nTargets);         % Antenna Array Delay [nRxAnts x nTargets]

        for t = 1:nTargets
            aryDelay(:,t)  = rxAntAry.*sind(azi(t))/c;    % Antenna Array Delay [nRxAnts x 1]
            steeringVec{t} = exp(2j*pi*fc*aryDelay(:,t)); % Array Azimuth Steering/Response Vector [nRxAnts x 1]
        end

    end

    steeringVec = cat(2,steeringVec{:}); % [nRxAnts x nTargets]

    radarEstParams.eleRes        = eleRes; 
    radarEstParams.aziRes        = aziRes;
    radarEstParams.antFFT        = antFFT;
    radarEstParams.RxSteeringVec = steeringVec;

    %% Restore Targets' Real Position Configuration
    % CFAR detection zone
    radarEstParams.cfarEstZone = simParams.cfarEstZone;
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

