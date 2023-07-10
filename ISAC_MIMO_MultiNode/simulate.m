function [gNBEstResults,gNBEstRMSE,gNBComResults] = simulate(gNBs,algParams,channels,antConfig,parSimuFlag)
%% System-Level 5G-Based ISAC Simulator 

% Simulate multi-node integrated sensing and communcation network (focused only on PHY layer). 

% Author: D.S.Xue, Key Laboratory of Universal Wireless Communications,
% Ministry of Education, BUPT.

    %% gNB communication and sensing results
    numgNB = numel(gNBs);
    [gNBComResults,gNBEstResults,gNBEstRMSE] = deal(cell(1,numgNB));

    %% ISAC Multi-Node Simulation
    if ~parSimuFlag % Local simulation
        for n = 1:numgNB 
            [gNBEstResults{n},gNBEstRMSE{n},gNBComResults{n}] = ISAC.singlegNBSimulation(gNBs(n),algParams,channels,antConfig);
        end
    else % Parallel simulation
        parfor n = 1:numgNB
            [gNBEstResults{n},gNBEstRMSE{n},gNBComResults{n}] = ISAC.singlegNBSimulation(gNBs(n),algParams,channels,antConfig);
        end
    end

end
