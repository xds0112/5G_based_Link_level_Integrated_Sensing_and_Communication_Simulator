classdef poissonUE < networkElements.networkElementsWithPosition
    %UE 
    %   
    
    properties
        
        % Height of the antenna (m)
        height = 1.5;

        % Position list of all UEs
        positionList 
        
    end
    
    methods
        function obj = poissonUE()
            %UE 
            %   
            obj@networkElements.networkElementsWithPosition()

        end

        function handle = plot(obj, timeIndex, color)
            % plot plots the User at a specific time Index in a color
            % either color triplet [r g b] or 'm', 'y' ...
            for elem = obj
                p = elem.positionList(:,timeIndex);
                hold on;
                handle = scatter3(p(1), p(2), p(3), 10, color, 'filled');
                hold off;
            end
        end
        
    end
end

