classdef basicTarget < networkElements.networkElementsWithPosition
    %BASICTARGET 
    %   
    
    properties

        % Radar cross section
        rcs

        % Radial velocity
        velocity

    end
    
    methods
        function obj = basicTarget()
            %BASICTARGET 
            %   

            obj@networkElements.networkElementsWithPosition()

        end
        
    end
end

