function estResults = hRadar3DFFT(radarEstParams, cfar, rxGrid, txGrid, txArray, wtx)
%%3D-FFT Algorithm for Range, Velocity and Angle Estimation.
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
% wtx: precoding weights of Tx
%
% Output parameters: 
%
% estResults containing the estimated range, velocity, and angle
% for each target detected. The function also includes functions to plot results.
%
% Author: D.S Xue, Key Laboratory of Universal Wireless Communications,
% Ministry of Education, BUPT.

    %% Input Parameters
    nAnts   = size(rxGrid,3);
    nIFFT   = radarEstParams.nIFFT;
    nFFT    = radarEstParams.nFFT;
    nAntFFT = radarEstParams.antFFT;

    % Range and Doppler grid
    rngGrid = ((0:nIFFT-1)*radarEstParams.rRes)';        % [0,nIFFT-1]*rRes
    dopGrid = ((-nFFT/2:nFFT/2-1)*radarEstParams.vRes)'; % [-nFFT/2,nFFT/2-1]*vRes

    % CFAR
    cfarDetector = cfar.cfarDetector2D;
    CUTIdx       = cfar.CUTIdx;
    if ~strcmp(cfarDetector.OutputFormat,'Detection index')
        cfarDetector.OutputFormat = 'Detection index';
    end

    %% Simulation params initialization
    % 3D-FFT parameters
    detections  = cell(nAnts,1);
    steeringVec = NaN;

    % Estimated results
    estResults        = struct;
    estResults.rngEst = cell(nAnts,1);
    estResults.velEst = cell(nAnts,1);

    %% 3D-FFT Algorithm
    % Element-wise multiplication
    channelInfo = bsxfun(@times, rxGrid, pagectranspose(pagetranspose(txGrid)));  % [nSc x nSym x nAnts]

    % Select window
    [rngWin,dopWin] = selectWindow('kaiser');

    % Generate windowed RDM
    chlInfoWindowed = channelInfo.*rngWin;                                       % Apply window to the channel info matrix
    rngIFFT         = ifftshift(ifft(chlInfoWindowed,nIFFT,1).*sqrt(nIFFT));     % IDFT per columns, [nIFFT x nSym x nAnts]
    rngIFFTWindowed = rngIFFT.*dopWin;                                           % Apply window to the ranging matrix
    rdm             = fftshift(fft(rngIFFTWindowed,nFFT,2)./sqrt(nFFT));         % DFT per rows, [nIFFT x nFFT x nAnts]

    % Range and velocity estimation
    for r = 1:nAnts
        % CFAR detection
        detections{r} = cfarDetector(abs(rdm(:,:,r).^2),CUTIdx);

        % Range and velocity estimation
        for i = 1:size(detections{r},2) 
            if ~isempty(detections{r})

                % Detection indices
                rngIdx = detections{r}(1,i)-1;
                velIdx = detections{r}(2,i)-nFFT/2-1;

                % Assignment
                estResults(i).rngEst{r} = rngIdx.*radarEstParams.rRes;
                estResults(i).velEst{r} = velIdx.*radarEstParams.vRes;

                % Angle steering vector, [nAnts x nTargets]
                % Remove the effects of Tx precoding weights
                steeringVec(r,i) = rdm(detections{r}(1,i)-1,detections{r}(2,i)-1,r).*wtx(end,r);

             end
         end
    end

    % Mean estimation values of all antenna array elements
    ntgtDet = length(estResults); % Number of targets detected  
    for j = 1:ntgtDet
        estResults(j).rngEst = mean(cat(2,estResults(j).rngEst{:}),2);
        estResults(j).velEst = mean(cat(2,estResults(j).velEst{:}),2);
    end

    % Angle estimation
    [aziEst,eleEst] = deal(zeros(1,ntgtDet)); % Est values initialization

    if isa(txArray,'phased.NRRectangularPanelArray') % UPA

        for k = 1:ntgtDet
            nAryZ  = txArray.Size(1);     % Z Axis antenna array
            nAryY  = txArray.Size(2)*2;   % Y Axis antenna array
            aryMat = reshape(steeringVec(:,k),nAryZ,nAryY);
        
            aryZFFT   = fftshift(fft(aryMat,nAntFFT,1));                 % [nAntFFT x nAryY]
            [~,eIdx]  = max(abs(aryZFFT),[],1);                          % [1 x nAryY]
            eEst      = asind((eIdx-nAntFFT/2-1)*radarEstParams.eleRes); % [1 x nAryY]
            eleEst(k) = mean(eEst);
        
            aryYIFFT  = ifftshift(ifft(aryMat,nAntFFT,2));                               % [nAryZ x nAntFFT] 
            [~,aIdx]  = max(abs(aryYIFFT),[],2);                                         % [nAryZ x 1]
            aEst      = asind((aIdx-nAntFFT/2-1)*radarEstParams.aziRes/cosd(eleEst(k))); % [nAryZ x 1]     
            aziEst(k) = mean(aEst);

            % Assignment
            estResults(k).eleEst = eleEst(k);
            estResults(k).aziEst = aziEst(k);
        end
       
        % Mark complex values with NaN
        cpxIdx = aziEst ~= real(aziEst);
        [estResults(cpxIdx).rngEst,estResults(cpxIdx).velEst,...
            estResults(cpxIdx).eleEst,estResults(cpxIdx).aziEst] = deal(NaN);

    else % ULA

        aryFFT = fftshift(fft(steeringVec,nAntFFT),1);  % [nAntFFT x nTargets]

        for k = 1:ntgtDet
            [~,aIdx]  = max(abs(aryFFT(:,k)));
            aziEst(k) = asind((aIdx-nAntFFT/2-1)*radarEstParams.aziRes);
 
            % Assignment
            estResults(k).aziEst = aziEst(k);
        end
    
        % Mark complex values with NaN
        cpxIdx = aziEst ~= real(aziEst);
        [estResults(cpxIdx).rngEst,estResults(cpxIdx).velEst,...
            estResults(cpxIdx).aziEst] = deal(NaN);

    end 

    %% Plot Results
    % plot 2D-RDM (1st Rx antenna array element)
    plotRDM(1)

    % plot 3D-FFT spectra
    plot3DFFTSpectra(1,1,1)

    %% Local functions
    function [rngWin,dopWin] = selectWindow(winType)
        % Windows for sidelobe suppression: 'hamming, hann, blackman, 
        % kaiser, taylorwin, chebwin, barthannwin, gausswin, tukeywin'
        [nSc,nSym,~] = size(channelInfo);
        switch winType
            case 'hamming'      % Hamming window
                rngWin = repmat(hamming(nSc,3),[1 nSym]);
                dopWin = repmat(hamming(nSym,3),[1 nIFFT]).';
            case 'hann'         % Hanning window
                rngWin = repmat(hann(nSc,3),[1 nSym]);
                dopWin = repmat(hann(nSym,3),[1 nIFFT]).';
            case 'blackman'     % Blackman window
                rngWin = repmat(blackman(nSc,3),[1 nSym]);
                dopWin = repmat(blackman(nSym,3),[1 nIFFT]).';
            case 'kaiser'       % Kaiser window
                rngWin = repmat(kaiser(nSc,3),[1 nSym]);
                dopWin = repmat(kaiser(nSym,3),[1 nIFFT]).';
            case 'taylorwin'    % Taylor window
                rngWin = repmat(taylorwin(nSc,3),[1 nSym]);
                dopWin = repmat(taylorwin(nSym,3),[1 nIFFT]).';
            case 'chebwin'      % Chebyshev window
                rngWin = repmat(chebwin(nSc,3),[1 nSym]);
                dopWin = repmat(chebwin(nSym,3),[1 nIFFT]).';
            case 'barthannwin'  % Modified Bartlett-Hann window
                rngWin = repmat(barthannwin(nSc,3),[1 nSym]);
                dopWin = repmat(barthannwin(nSym,3),[1 nIFFT]).';
            case 'gausswin'     % Gaussian window
                rngWin = repmat(gausswin(nSc,3),[1 nSym]);
                dopWin = repmat(gausswin(nSym,3),[1 nIFFT]).';
            case 'tukeywin'     % Gaussian window
                rngWin = repmat(tukeywin(nSc,3),[1 nSym]);
                dopWin = repmat(tukeywin(nSym,3),[1 nIFFT]).';
            otherwise           % Default to Hamming window
                rngWin = repmat(hamming(nSc,3),[1 nSym]);
                dopWin = repmat(hamming(nIFFT,3),[1 nSym]).';
        end
    end

    function plotRDM(aryIdx)
    % plot 2D range-Doppler(velocity) map
        figure('Name','2D RDM')

        h = imagesc(dopGrid,rngGrid,mag2db(abs(rdm(:,:,aryIdx))));
        h.Parent.YDir = 'normal';
        colorbar

        title('Range-Doppler Map')
        xlabel('Radial Velocity (m/s)')
        ylabel('Range (m)')

    end

    function plot3DFFTSpectra(fastTimeIdx,slowTimeIdx,aryIdx)
    % Plot 3D-FFT spectra (in dB)  
        figure('Name','2D Estimation Results')
     
        t = tiledlayout(3,1,'TileSpacing','compact');
        title(t,'3D-FFT Estimation')
        ylabel(t,'FFT Spectra (dB)')

        % plot angle estimation 
        nexttile(1)
        aziGrid    = asind((-nAntFFT/2:nAntFFT/2-1)*radarEstParams.aziRes);
        aziFFT     = abs(sum(aryFFT,2));
        aziFFTNorm = aziFFT./max(aziFFT);
        aziFFTdB   = mag2db(aziFFTNorm);
        plot(aziGrid,aziFFTdB,'LineWidth',1);      
        title('DoA Estimation')
        xlabel('Azimuth (°)')
        xlim([-90 90])
        grid on

        % plot range estimation 
        nexttile(2)
        rngIFFT     = abs(ifftshift(rngIFFT(:,slowTimeIdx,aryIdx)));
        rngIFFTNorm = rngIFFT./max(rngIFFT);
        rngIFFTdB   = mag2db(rngIFFTNorm);
        plot(rngGrid,rngIFFTdB,'LineWidth',1);
        title('Range Estimation')
        xlabel('Range (m)')
        xlim([0 500])
        grid on

        % plot doppler/velocity estimation 
        nexttile(3)    
        % DFT per rows, [nSc x nFFT x nAnts]
        velFFT     = abs(fftshift(fft(chlInfoWindowed(fastTimeIdx,:,aryIdx),nFFT,2)./sqrt(nFFT)));
        velFFTNorm = velFFT./max(velFFT);
        velFFTdB   = mag2db(velFFTNorm);
        plot(dopGrid,velFFTdB,'LineWidth',1);
        title('Velocity(Doppler) Estimation')
        xlabel('Radial Velocity (m/s)')
        xlim([-50 50])
        grid on

    end
    
end