function estResults = fft2D(radarEstParams, cfar, rxGrid, txGrid)
%2D-FFT Algorithm for Range, Velocity and Angle Estimation.
%
% Input parameters:
%
% radarEstParams: structure containing radar system parameters 
% such as the number of IFFT/FFT points, range and velocity resolutions, and angle FFT size.
%
% cfarDetector: an object implementing the Constant False Alarm Rate (CFAR) detection algorithm.
%
% CUTIdx: index of the cells under test (CUT)
%
% rxGrid: M-by-N-by-P matrix representing the received signal 
% at P antenna elements from N samples over M chirp sequences.
%
% txGrid: M-by-N-by-P matrix representing the transmitted signal 
% at P antenna elements from N samples over M chirp sequences.
%
% txArray: phased array System object™ or a NR Rectangular Panel Array (URA) System object™
%
%
% Output parameters: 
%
% estResults containing the estimated range, velocity, and angle
% for each target detected. The function also includes functions to plot results.
%
% Author: D.S Xue, Key Laboratory of Universal Wireless Communications,
% Ministry of Education, BUPT.

    %% Parameters
    [nSc, nSym, nAnts] = size(rxGrid);
    nIFFT = radarEstParams.nIFFT;
    nFFT  = radarEstParams.nFFT;

    % Estimated results
    estResults = struct;

    %% DoA Estimation
    % Array correlation matrix
    rxGridReshaped = reshape(rxGrid, nSc*nSym, nAnts)'; % [nAnts x nSc*nSym]
    Ra = rxGridReshaped*rxGridReshaped'./(nSc*nSym);    % [nAnts x nAnts]

    % MUSIC method
    [numDets, aziEst, eleEst] = sensing.estimation.doaEstimation.music(radarEstParams, Ra);

    % Assignment
    for i = 1:numDets
        estResults(i).aziEst = aziEst(i);
        estResults(i).eleEst = eleEst(i);
    end

    %% 2D-FFT Algorithm
    % Element-wise multiplication
    channelInfo = bsxfun(@times, rxGrid, pagectranspose(pagetranspose(txGrid)));  % [nSc x nSym x nAnts]

    % Select window
    [rngWin, dopWin] = selectWindow('kaiser');

    % Generate windowed RDM
    chlInfo = channelInfo.*rngWin;                             % apply window to the channel info matrix
    rngIFFT = ifftshift(ifft(chlInfo, nIFFT, 1).*sqrt(nIFFT)); % IDFT per columns, [nIFFT x nSym x nAnts]
    rngIFFT = rngIFFT.*dopWin;                                 % apply window to the ranging matrix
    rdm     = fftshift(fft(rngIFFT, nFFT, 2)./sqrt(nFFT));     % DFT per rows, [nIFFT x nFFT x nAnts]

    % Range and velocity estimation
    % CFAR detection
    cfarDetector = cfar.cfarDetector2D;
    CUTIdx       = cfar.CUTIdx;
    if ~strcmp(cfarDetector.OutputFormat,'Detection index')
        cfarDetector.OutputFormat = 'Detection index';
    end
    cfarDetector.NumDetections = numDets; % maximum number of detections to report

    rdResponse = abs(rdm(:,:,1)).^2;
    detections = cfarDetector(rdResponse, CUTIdx);
    nDetecions = size(detections, 2);

    % Restore estimation values
    [rngEst, velEst, peaks] = deal(zeros(1, nDetecions));

    if ~isempty(detections)

        for i = 1:nDetecions

            % Peak levels
            peaks(i) = rdResponse(detections(1,i), detections(2,i), 1);

            % Detection indices
            rngIdx = detections(1,i)-1;
            velIdx = detections(2,i)-nFFT/2-1;

            % Range and velocity estimation
            rngEst(i) = rngIdx.*radarEstParams.rRes;
            velEst(i) = velIdx.*radarEstParams.vRes;

        end

    end

    % Sort estimations by descending order
    [~, idx] = sort(peaks, 'descend');
    [rngEst(:), velEst(:)] = deal(rngEst(idx), velEst(idx));
    for i = 1:nDetecions
        % Assignment
        estResults(i).rngEst = rngEst(i);
        estResults(i).velEst = velEst(i);
    end

    %% Plot Results
    % plot 2D-RDM (1st Rx antenna array element)
    plotRDM(1)

    % Uncomment to plot 2D-FFT spectra
    % plotFFTSpectra(1,1,1)

    %% Local functions
    function [rngWin, dopWin] = selectWindow(winType)
        % Windows for sidelobe suppression: 'hamming, hann, blackman, 
        % kaiser, taylorwin, chebwin, barthannwin, gausswin, tukeywin'
        switch winType
            case 'hamming'      % Hamming window
                rngWin = repmat(hamming(nSc), [1 nSym]);
                dopWin = repmat(hamming(nIFFT), [1 nSym]);
            case 'hann'         % Hanning window
                rngWin = repmat(hann(nSc), [1 nSym]);
                dopWin = repmat(hann(nIFFT), [1 nSym]);
            case 'blackman'     % Blackman window
                rngWin = repmat(blackman(nSc), [1 nSym]);
                dopWin = repmat(blackman(nIFFT), [1 nSym]);
            case 'kaiser'       % Kaiser window
                rngWin = repmat(kaiser(nSc, 3), [1 nSym]);
                dopWin = repmat(kaiser(nIFFT, 3), [1 nSym]);
            case 'taylorwin'    % Taylor window
                rngWin = repmat(taylorwin(nSc, 4, -30), [1 nSym]);
                dopWin = repmat(taylorwin(nIFFT, 4, -30), [1 nSym]);
            case 'chebwin'      % Chebyshev window
                rngWin = repmat(chebwin(nSc, 50), [1 nSym]);
                dopWin = repmat(chebwin(nIFFT, 50), [1 nSym]);
            case 'barthannwin'  % Modified Bartlett-Hann window
                rngWin = repmat(barthannwin(nSc), [1 nSym]);
                dopWin = repmat(barthannwin(nIFFT), [1 nSym]);
            case 'gausswin'     % Gaussian window
                rngWin = repmat(gausswin(nSc, 2.5), [1 nSym]);
                dopWin = repmat(gausswin(nIFFT, 2.5), [1 nSym]);
            case 'tukeywin'     % tukey (tapered cosine) window
                rngWin = repmat(tukeywin(nSc, .5), [1 nSym]);
                dopWin = repmat(tukeywin(nIFFT, .5), [1 nSym]);
            otherwise           % Default to Hamming window
                rngWin = repmat(hamming(nSc), [1 nSym]);
                dopWin = repmat(hamming(nIFFT), [1 nSym]);
        end
    end

    function plotRDM(aryIdx)
    % plot 2D range-Doppler(velocity) map
        figure('Name','2D RDM')

        % Range and Doppler grid for plotting
        rngGrid = ((0:nIFFT-1)*radarEstParams.rRes)';        % [0, nIFFT-1]*rRes
        dopGrid = ((-nFFT/2:nFFT/2-1)*radarEstParams.vRes)'; % [-nFFT/2, nFFT/2-1]*vRes

        rdmdB = mag2db(abs(rdm(:,:,aryIdx)));
        h = imagesc(dopGrid, rngGrid, rdmdB);
        h.Parent.YDir = 'normal';

        title('Range-Doppler Map')
        xlabel('Radial Velocity (m/s)')
        ylabel('Range (m)')

    end

    function plotFFTSpectra(fastTimeIdx, slowTimeIdx, aryIdx)
    % Plot 2D-FFT spectra (in dB)  
        figure('Name', '2D FFT Results')
     
        t = tiledlayout(2, 1, 'TileSpacing', 'compact');
        title(t, '2D-FFT Estimation')
        ylabel(t, 'FFT Spectra (dB)')

        % Range and Doppler grid for plotting
        rngGrid = ((0:nIFFT-1)*radarEstParams.rRes)';        % [0, nIFFT-1]*rRes
        dopGrid = ((-nFFT/2:nFFT/2-1)*radarEstParams.vRes)'; % [-nFFT/2, nFFT/2-1]*vRes
        
        % plot range spectrum 
        nexttile(1)
        rngIFFTPlot = abs(ifftshift(rngIFFT(:, slowTimeIdx, aryIdx)));
        rngIFFTNorm = rngIFFTPlot./max(rngIFFTPlot);
        rngIFFTdB   = mag2db(rngIFFTNorm);
        plot(rngGrid, rngIFFTdB, 'LineWidth', 1);
        title('Range Estimation')
        xlabel('Range (m)')
        grid on

        % plot Doppler/velocity spectrum 
        nexttile(2)    
        % DFT per rows, [nSc x nFFT x nAnts]
        velFFTPlot = abs(fftshift(fft(chlInfo(fastTimeIdx, :, aryIdx), nFFT, 2)./sqrt(nFFT)));
        velFFTNorm = velFFTPlot./max(velFFTPlot);
        velFFTdB   = mag2db(velFFTNorm);
        plot(dopGrid, velFFTdB, 'LineWidth', 1);
        title('Velocity(Doppler) Estimation')
        xlabel('Radial Velocity (m/s)')
        grid on

    end
    
end
