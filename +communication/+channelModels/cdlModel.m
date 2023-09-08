classdef cdlModel
    %CHANNELMODEL 
    %   Communication channel model
    
    properties

        % Carrier frequency
        carrierFrequency

        % NLoS: 'CDL-A', 'CDL-B', 'CDL-C'
        % LoS: 'CDL-D', 'CDL-E'
        delayProfile

        % Attached UEs
        attachedUEs

        % Antenna configuration
        antConfig

        % Topology parameters
        topoParams

    end

    properties (Dependent)

        % Multi-user channel models
        models

    end
    
    methods
        function obj = cdlModel()
            %CHANNELMODEL  
        end

        function models = get.models(obj)

            % CDL channel models
            delaySpread = 300e-9;
            maximumDopplerShift = 5;
            models = communication.channelModels.multiUserChannels(obj.delayProfile, ...
                obj.carrierFrequency, delaySpread, maximumDopplerShift, ...
                obj.antConfig.bsTxAntSize, obj.antConfig.ueAntSizes, obj.topoParams);

        end
        
    end
end

