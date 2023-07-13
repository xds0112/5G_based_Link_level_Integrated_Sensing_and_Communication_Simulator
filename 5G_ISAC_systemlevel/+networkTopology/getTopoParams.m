function topParams = getTopoParams(bsParams)
%GETTOPOPARAMS
%   Calculate the position of the target relative to the base station

        for ibs = 1:numel(bsParams)

            bsPos = bsParams(ibs).position;

            % Attached targets
            tgtParams = bsParams(ibs).attachedTgts;
            numTgt = numel(tgtParams);

            % Restore positions and velocities
            tgtPos = zeros(3,numTgt);
            tgtVel = zeros(1,numTgt);
           
            for itg = 1:numTgt

                tgtPos(:,itg) = tgtParams(itg).position;
                tgtVel(itg)   = tgtParams(itg).velocity;

            end

        end

        % Calculate relative coords
        coords = tgtPos - repmat(bsPos',1,numTgt);

        % Calculate relative distances and angles
        [azimuth, elevation, range] = cart2sph(coords(1,:), coords(2,:), coords(3,:));

        % Convert the angles to degrees
        [azimuth, elevation] = deal(rad2deg(azimuth), rad2deg(elevation));

        % Topology params assignment
        topParams = struct;
        topParams.range     = range;
        topParams.velocity  = tgtVel;
        topParams.azimuth   = azimuth;
        topParams.elevation = elevation;

end

