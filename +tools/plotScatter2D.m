function plthandle = plotScatter2D(position, size, color)
% plot 2D scatter of an object
% input: position, (x,y) coordinates
%        size, scatter size
%        color, scatter color
%
% output: plot handle

    hold on

    plthandle = scatter(position(1), position(2), size, color, 'filled');

    hold off
end