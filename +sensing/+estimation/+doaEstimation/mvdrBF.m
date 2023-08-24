function aziEst = mvdrBF(radarEstParams, Ra)
%MVDRBF Minimum variance distortionless response (MVDR) beamformer for DoA estimation
%  Author: D.S Xue, Key Laboratory of Universal Wireless Communications,
% Ministry of Education, BUPT.

    % Array parameters\
    nAnts           = size(Ra, 1);                     % number of antenna elements
    d               = .5;                              % antenna array element spacing, normally set to 0.5
    scanGranularity = radarEstParams.scanGranularity;  % beam scan granularity, in degree
    aMax            = radarEstParams.scanScale;        % beam scan scale, in degree
    aSteps          = floor((aMax+1)/scanGranularity); % beam scan steps

    % Minimum variance distortionless response (MVDR) beamforming method  
    Pmvdr = zeros(1, aSteps);
    for a = 1:aSteps
        scanAngle = (a-1)*scanGranularity - aMax/2;
        aa        = exp(-2j.*pi.*sind(scanAngle).*d.*(0:1:nAnts-1)).'; % angle steering vector, [1 x nAnts]
        Pmvdr(a)  = 1./(aa'*Ra^-1*aa);
    end
    
    % Normalization
    Pmvdr     = abs(Pmvdr);
    PmvdrNorm = Pmvdr./max(Pmvdr);
    PmvdrdB   = mag2db(PmvdrNorm);
    
    % DoA estimation
    [~, aIdx] = findpeaks(PmvdrdB, 'MinPeakHeight', -5, 'SortStr', 'descend');
    aziEst = (aIdx-1)*scanGranularity - aMax/2;

    % Plot
    plotAngularSpectrum

    %% Local Functions
    function plotAngularSpectrum()
    % Plot angular spectrum (in dB)  
        figure('Name', 'Angular Spectrum')

        % Angular grid for plotting
        aziGrid = linspace(-aMax/2, aMax/2, aSteps); % [-aMax/2, aMax/2]

        % plot DoA spectrum 
        plot(aziGrid, PmvdrdB, 'LineWidth', 1);

        title('DoA Estimation using MVDR Method')
        ylabel('Angular Spectrum (dB)')
        xlabel('DoA (°)')
        xlim([-aMax/2 aMax/2])
        grid on

    end

end