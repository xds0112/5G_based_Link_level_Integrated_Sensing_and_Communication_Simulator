function  plotTopology(bsParams, estResults)
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
        estPos = [x, y, z] + bsPos;

        % Plot
        plot3DTopology(bsPos, uePos, tgtPos, estPos)

    else % ULA model

        [x, y] = pol2cart(deg2rad(aziEst), rngEst);
        estPos = [x, y] + bsPos(1:2);

        % Plot
        plot2DTopology(bsPos, uePos, tgtPos, estPos)

    end

    %% Plot

    function plot3DTopology(bsPos, uePos, tgtPos, estPos)
        figure('Name', '3D Simulation Topology')

        bsPlot  = tools.plotScatter3D(bsPos, 50, tools.colors.darkRed);
        uePlot  = tools.plotScatter3D(uePos, 15, tools.colors.darkBlue);
        tgtPlot = tools.plotScatter3D(tgtPos, 15, tools.colors.darkGreen);
        for i = 1:size(estPos, 1)
            estPlot = tools.plotScatter3D(estPos(i,:), 15, tools.colors.lightRed);
        end
    
        legend([bsPlot, uePlot, tgtPlot, estPlot], {'Base Station' 'UEs' 'Targets' 'Estimated Targets'}, 'Location', 'best')
    
        grid on

        xlabel('x axis (m)')
        ylabel('y axis (m)')
        zlabel('z axis (m)')
        xlim([-500 500])
        ylim([-500 500])
        zlim([0 100])

    end

    function plot2DTopology(bsPos, uePos, tgtPos, estPos)
        figure('Name', '2D Simulation Topology')

        bsPlot  = tools.plotScatter2D(bsPos(1:2), 50, tools.colors.darkRed);
        uePlot  = tools.plotScatter2D(uePos(1:2), 25, tools.colors.darkBlue);
        tgtPlot = tools.plotScatter2D(tgtPos(1:2), 25, tools.colors.darkGreen);
        for i = 1:size(estPos, 1)
            estPlot = tools.plotScatter2D(estPos(i,:), 15, tools.colors.lightRed);
        end
    
        legend([bsPlot, uePlot, tgtPlot, estPlot], {'Base Station' 'UEs' 'Targets' 'Estimated Targets'}, 'Location', 'best')
    
        grid on

        xlabel('x axis (m)')
        ylabel('y axis (m)')
        xlim([-500 500])
        ylim([-500 500])

    end

end

