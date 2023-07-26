function topoParams = getTopoParams(bsParams)
%GETTOPOPARAMS
%   Calculate the position of the target relative to the base station

        % Topology params assignment
        numBSs = numel(bsParams);
        topoParams = repmat(struct,1,numBSs);

        for ibs = 1:numBSs

            bsPos = bsParams(ibs).position;
            
            % Attached UEs
            ueParams = bsParams(ibs).attachedUEs;
            numUEs = numel(ueParams);

            % Restore UEs' positions
            uePos = zeros(3,numUEs);

            for iue = 1:numUEs
                uePos(:,iue) = ueParams(iue).position;
            end

            % Attached targets
            tgtParams = bsParams(ibs).attachedTgts;
            numTgts = numel(tgtParams);

            % Restore targets' positions and velocities
            tgtPos = zeros(3,numTgts);
            tgtVel = zeros(1,numTgts);
      
            for itg = 1:numTgts
                tgtPos(:,itg) = tgtParams(itg).position;
                tgtVel(itg)   = tgtParams(itg).velocity;
            end

            % Calculate relative coords
            coordsUEs  = uePos - repmat(bsPos',1,numUEs);
            coordsTgts = tgtPos - repmat(bsPos',1,numTgts);
    
            % Calculate relative distances and angles
            [azimuthUEs, elevationUEs, rangeUEs] = cart2sph(coordsUEs(1,:), coordsUEs(2,:), coordsUEs(3,:));
            [azimuthTgts, elevationTgts, rangeTgts] = cart2sph(coordsTgts(1,:), coordsTgts(2,:), coordsTgts(3,:));
    
            % Convert the angles to degrees
            [azimuthUEs, elevationUEs]   = deal(rad2deg(azimuthUEs), rad2deg(elevationUEs));
            [azimuthTgts, elevationTgts] = deal(rad2deg(azimuthTgts), rad2deg(elevationTgts));
    
            % UEs
            topoParams(ibs).rangeUEs     = rangeUEs;
            topoParams(ibs).azimuthUEs   = azimuthUEs;
            topoParams(ibs).elevationUEs = elevationUEs;
            % Targets
            topoParams(ibs).rangeTgts     = rangeTgts;
            topoParams(ibs).azimuthTgts   = azimuthTgts;
            topoParams(ibs).elevationTgts = elevationTgts;
            topoParams(ibs).velocityTgts  = tgtVel;
 
        end

end

