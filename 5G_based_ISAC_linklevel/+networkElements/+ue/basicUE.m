classdef basicUE < networkElements.networkElementsWithPosition
    %UE 
    %   
    
    properties
        
        % Height of the antenna (m)
        height = 1.5;
        
    end
    
    methods
        function obj = basicUE()
            %UE 
            %   
            obj@networkElements.networkElementsWithPosition()

        end
        
    end
end

