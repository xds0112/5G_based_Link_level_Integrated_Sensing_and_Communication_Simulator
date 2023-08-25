function aziEst = digitalBF(radarEstParams, Ra)
%DIGITALBF Digital beamformer (DBF) for DoA estimation
%
%  Author: D.S Xue, Key Laboratory of Universal Wireless Communications,
% Ministry of Education, BUPT.

    % Array parameters
    nAnts           = size(Ra, 1);                     % number of antenna elements
    d               = .5;                              % antenna array element spacing, normally set to 0.5
    scanGranularity = radarEstParams.scanGranularity;  % beam scan granularity, in degree
    aMax            = radarEstParams.scanScale;        % beam scan scale, in degree
    aSteps          = floor((aMax+1)/scanGranularity); % beam scan steps

    % Digital beamforming method  
    Pdbf = zeros(1, aSteps);
    for a = 1:aSteps
        scanAngle = (a-1)*scanGranularity - aMax/2;
        aa        = exp(-2j.*pi.*sind(scanAngle).*d.*(0:1:nAnts-1)).'; % angle steering vector, [1 x nAnts]
        Pdbf(a)   = aa'*Ra*aa;
    end
    
    % Normalization
    Pdbf     = abs(Pdbf);
    PdbfNorm = Pdbf./max(Pdbf);
    PdbfdB   = mag2db(PdbfNorm);
    
    % DoA estimation
    [~, aIdx] = findpeaks(PdbfdB, 'MinPeakHeight', -5, 'SortStr', 'descend');
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
        plot(aziGrid, PdbfdB, 'LineWidth', 1);

        title('DoA Estimation using Digital Beamforming Method')
        ylabel('Angular Spectrum (dB)')
        xlabel('Azimuth (°)')
        xlim([-aMax/2 aMax/2])
        grid on

    end

end

