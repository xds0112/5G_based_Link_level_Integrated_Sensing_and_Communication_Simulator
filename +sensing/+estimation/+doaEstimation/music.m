function [numTgtsEst, aziEst] = music(radarEstParams, Ra)
%MUSIC Multiple signal classification (MUSIC) algorithm for DoA estimation
%
%  Author: D.S Xue, Key Laboratory of Universal Wireless Communications,
% Ministry of Education, BUPT.

    % Array parameters
    nAnts           = size(Ra, 1);                     % number of antenna elements
    d               = .5;                              % antenna array element spacing, normally set to 0.5
    scanGranularity = radarEstParams.scanGranularity;  % beam scan granularity, in degree
    aMax            = radarEstParams.scanScale;        % beam scan scale, in degree
    aSteps          = floor((aMax+1)/scanGranularity); % beam scan steps

    % Eigenvalue decomposition
    % Ua: orthogonal eigen matrix, 
    % Sa: real-value eigenvalue diagonal matrix in descending order
    % Va: a vector taking the diagonal values of Sa
    % L: The number of targets in the DoA of interest
    [Ua, Sa] = eig(Ra);
    Va       = real(diag(Sa));
    L        = determineNumTargets(Va);
    [~, Ia]  = sort(Va, 'descend');
    Ua       = Ua(:,Ia);
    Uan      = Ua(:,L+1:end);

    Pmusic = zeros(1, aSteps); % azimuth spectrum
    % Angle spectrum
    for a = 1:aSteps
        searchAngle = (a-1)*scanGranularity-aMax/2;
        aa          = exp(-2j.*pi.*sind(searchAngle).*d.*(0:1:nAnts-1)).';  % angle steering vector
        Pmusic(a)   = 1./((aa'*Uan)*(Uan'*aa));
    end
    
    % Normalization
    Pmusic     = abs(Pmusic);
    PmusicNorm = Pmusic./max(Pmusic);
    PmusicdB   = mag2db(PmusicNorm);

    % Assignment
    [~, azi] = findpeaks(PmusicdB, 'MinPeakHeight', -5, 'SortStr', 'descend');
    aziEst = (azi-1)*scanGranularity-aMax/2;
    numTgtsEst = L; % number of targets estimated

    % Plot
    plotAngularSpectrum

    %% Local Functions
    function L = determineNumTargets(V)
    % Determine the number of detected targets

        % [∆v]i = [v]i - [v]i+1
        deltaV = -diff(V);

        % The mean value of the latter half of ∆v
        n = length(deltaV);
        halfMeanV = mean(deltaV(ceil((n+1)./2):1:end));

        % ε, the parameter used to avoid false 
        % detection cause by a small error
        epsilon = 1;
        % L = argmax [∆v] > (1 + ε)*halfMeanV
        [~,L] = max(deltaV - (1+epsilon).*halfMeanV);
        
    end

    function plotAngularSpectrum()
    % Plot angular spectrum (in dB)  
        figure('Name', 'Angular Spectrum')

        % Angular grid for plotting
        aziGrid = linspace(-aMax/2, aMax/2, aSteps); % [-aMax/2, aMax/2]

        % plot DoA spectrum 
        plot(aziGrid, PmusicdB, 'LineWidth', 1);

        title('DoA Estimation using MUSIC Method')
        ylabel('Angular Spectrum (dB)')
        xlabel('Azimuth (°)')
        xlim([-aMax/2 aMax/2])
        grid on

    end

end

