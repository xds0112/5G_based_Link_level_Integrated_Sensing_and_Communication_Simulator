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

        % Multi-user channel models
        model 

    end
    
    methods
        function obj = cdlModel()
            %CHANNELMODEL  
        end

        function obj = updateChannel(obj)

            % CDL channel models
            delaySpread = 300e-9;
            maximumDopplerShift = 5;
            obj.model = communication.channelModels.multiUserChannels(obj.delayProfile, ...
                obj.carrierFrequency, delaySpread, maximumDopplerShift, ...
                obj.antConfig.bsAntSize, obj.antConfig.ueAntSizes, obj.topoParams);

        end
        
    end
end

