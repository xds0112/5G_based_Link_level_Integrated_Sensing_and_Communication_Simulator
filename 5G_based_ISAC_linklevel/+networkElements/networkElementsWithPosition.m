classdef networkElementsWithPosition
    %NETWORKELEMENTSWITHPOSITION 
    %   
    
    properties
        % Element identity
        ID = 1

        % Three-dimensional Cartesian coordinates,
        % [x y z] in meters
        position = [0 0 0]
    end
    
    methods
        function obj = networkElementsWithPosition(ID,coordinates)
            %NETWORKELEMENTSWITHPOSITION 
            %  
            if nargin > 0
                obj.ID       = ID;
                obj.position = coordinates;
            end
        end
        
    end
end

