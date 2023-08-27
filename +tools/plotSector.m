function pltHandle = plotSector(originPos, angle1, angle2, color)
% Plot the specified sector of a gNB
% Input: (x0, y0) is the origin of the sector,
%        (angle1, angle2) represents the angular range of the sector, in degrees.

        [angle1, angle2] = deal(deg2rad(angle1), deg2rad(angle2));

        xLow = 1e3*[originPos(1), cos(angle1)]; % x coordinates of lower bound
        yLow = 1e3*[originPos(2), sin(angle1)]; % y coordinates of lower bound

        xUp = 1e3*[originPos(1), cos(angle2)]; % x coordinates of upper bound
        yUp = 1e3*[originPos(2), sin(angle2)]; % y coordinates of upper bound

        hold on
        pltHandle1 = plot(xLow, yLow, 'LineWidth', 1, 'Color', color); % plot handle of lower bound
        hold on
        pltHandle2 = plot(xUp, yUp, 'LineWidth', 1, 'Color', color);   % plot handle of upper bound
        hold off

        pltHandle = [pltHandle1 pltHandle2];

end

