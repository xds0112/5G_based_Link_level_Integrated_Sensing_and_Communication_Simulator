function steeringVector = getSteeringVector(array, ele, azi, lambda)
%GETSTEERINGVECTOR 
% Input parameters:
%   array: phased array System object™ or a NR Rectangular Panel Array (URA) System object™
%   ele: elevation angle of target (theta), [1 x nTargets]
%   azi: azimuth angle of target (phi), [1 x nTargets]
%   lambda: signal wavelength
%
% Outpur parameters:
%   steeringVector: antenna array steering vector, [nRxAnts x nTargets]
%
%
% Author: D.S Xue, Key Laboratory of Universal Wireless Communications,
% Ministry of Education, BUPT.

    nTargets = numel(ele);
    steeringVector = cell(1, nTargets);

    if isa(array, 'phased.NRRectangularPanelArray') % UPA model

        spacingX = array.Spacing(1);           % array X-axis element spacing
        spacingY = array.Spacing(2);           % array Y-axis element spacing
        nAntsX   = array.Size(1);              % array X-axis element number
        nAntsY   = array.Size(2);              % array Y-axis element number
        nAnts    = nAntsX*nAntsY;              % array element number 
        antAryX  = (0:1:nAntsX-1)*spacingX;    % array X-axis element indices, [1 x nRxAntsX]
        antAryY  = ((0:1:nAntsY-1)*spacingY)'; % array Y-axis element indices, [nRxAntsY x 1]

        % UPA steering vector, defined in the spheric coordinate system
        aUPA = @(ph, th, m, n)exp(2j*pi*sind(th)*(m*cosd(ph) + n*sind(ph))/lambda);
        
        for t = 1:nTargets
            upaSteeringVec = aUPA(azi(t), ele(t), antAryX, antAryY);
            steeringVector{t} = reshape(upaSteeringVec, nAnts, 1);
        end
    
    else  % ULA model

        spacing = array.ElementSpacing;     % array element spacing
        nAnts   = array.NumElements;        % array element number
        antAry  = ((0:1:nAnts-1)*spacing)'; % array element, [nRxAnts x 1]

        % ULA steering vector
        aULA = @(ph, m)exp(2j*pi*m*sind(ph)/lambda);

        for t = 1:nTargets
            ulaSteeringVec = aULA(azi(t), antAry);
            steeringVector{t} = ulaSteeringVec;
        end

    end

    % Convert cell to matrix
    steeringVector = cat(2, steeringVector{:}); % [nRxAnts x nTargets]

end

