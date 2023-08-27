function plotTopology(bsParams, estResults)
%PLOTTOPOLOGY plot network topology
%   plot the position of base station, UEs, and targets in the 3D layout

    % Base station position    
    bsPos = bsParams.position;
        
    % Attached UEs
    ueParams = bsParams.attachedUEs;
    numUEs = numel(ueParams);

    % Restore UEs' positions
    uePos = zeros(3, numUEs);
    for iue = 1:numUEs
        uePos(:, iue) = ueParams(iue).position;
    end

    % Attached targets
    tgtParams = bsParams.attachedTgts;
    numTgts = numel(tgtParams);

    % Restore targets' positions
    tgtPos = zeros(3, numTgts);
    for itg = 1:numTgts
        tgtPos(:, itg) = tgtParams(itg).position;
    end

    % Estimated positions
    aziEst = estResults.aziEst;
    eleEst = estResults.eleEst;
    rngEst = estResults.rngEst;

    if ~any(isnan(eleEst)) % UPA model

        % Convert to relative cartesian coordinates
        [x, y, z] = sph2cart(deg2rad(aziEst), deg2rad(eleEst), rngEst);
        estPos = ([x, y, z] + bsPos)';

        % Plot
        plot3DTopology(bsPos, uePos, tgtPos, estPos)

    else % ULA model

        [x, y] = pol2cart(deg2rad(aziEst), rngEst);
        estPos = ([x, y] + bsPos(1:2))';

        % Plot
        plot2DTopology(bsPos, uePos, tgtPos, estPos)

    end

    %% Plot

    function plot3DTopology(bsPos, uePos, tgtPos, estPos)
        figure('Name', '3D Simulation Topology')

        gNBPlot = tools.plotScatter3D(bsPos, 50, tools.colors.darkRed);
        tools.plotSector(bsPos, -60, 60, tools.colors.darkGrey);
        for u = 1:size(uePos, 2)
            uePlot = tools.plotScatter3D(uePos(:,u), 15, tools.colors.darkBlue);
        end
        for t = 1:size(tgtPos, 2)
            tgtPlot = tools.plotScatter3D(tgtPos(:,t), 15, tools.colors.darkGreen);
        end
        for i = 1:size(estPos, 2)
            estPlot = tools.plotScatter3D(estPos(:,i), 15, tools.colors.lightRed);
        end
    
        legend([gNBPlot, uePlot, tgtPlot, estPlot], {'gNB' 'UEs' 'Targets' 'Estimated Targets'}, 'Location', 'best')
    
        grid on

        xlabel('x axis (m)')
        ylabel('y axis (m)')
        zlabel('z axis (m)')
        xlim([0 500])
        ylim([-500 500])
        zlim([0 100])

    end

    function plot2DTopology(bsPos, uePos, tgtPos, estPos)
        figure('Name', '2D Simulation Topology')

        gNBPlot = tools.plotScatter2D(bsPos(1:2), 50, tools.colors.darkRed);
        tools.plotSector(bsPos, -60, 60, tools.colors.darkGrey);

        for u = 1:size(uePos, 2)
            uePlot  = tools.plotScatter2D(uePos(1:2,u), 25, tools.colors.darkBlue);
        end
        for t = 1:size(tgtPos, 2)
            tgtPlot = tools.plotScatter2D(tgtPos(1:2,t), 25, tools.colors.darkGreen);
        end
        for i = 1:size(estPos, 2)
            estPlot = tools.plotScatter2D(estPos(:,i), 15, tools.colors.lightRed);
        end
    
        legend([gNBPlot, uePlot, tgtPlot, estPlot], {'gNB' 'UEs' 'Targets' 'Estimated Targets'}, 'Location', 'best')
    
        grid on

        xlabel('x axis (m)')
        ylabel('y axis (m)')
        xlim([0 500])
        ylim([-500 500])

    end

end

