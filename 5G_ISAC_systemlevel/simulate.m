function [bsComResults, bsEstResults, bsEstRMSE] = simulate(simulationTime, BSs, tops, parSimuFlag)
%% System-Level 5G-based ISAC Simulator 

% Simulate multi-node integrated sensing and communcation network. 

% Author: D.S.Xue, Key Laboratory of Universal Wireless Communications,
% Ministry of Education, BUPT.

    %% Copy Right Printing

    currentTime = datetime('now');

    fprintf('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n');
    fprintf('Copyright (C) 2023 Beijing University of Posts and Telecommunications\n');
    fprintf('All rights reserved.\n');
    fprintf('\n');
    fprintf('System-level ISAC Simulator v1.0\n');
    fprintf('Author: Dongsheng Xue\n');
    fprintf('Date: %s\n', char(currentTime));
    fprintf('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n');

    %% gNB communication and sensing results
    numBS = numel(BSs);
    [bsComResults,bsEstResults,bsEstRMSE] = deal(cell(1,numBS));

    %% ISAC Multi-Node Simulation
    if ~parSimuFlag % Local simulation
        for n = 1:numBS 
            [bsComResults{n}, bsEstResults{n}, bsEstRMSE{n}] = ISAC.singleBSSimulation(simulationTime, BSs(n), tops);
        end
    else            % Parallel simulation
        parfor n = 1:numBS
            [bsComResults{n}, bsEstResults{n}, bsEstRMSE{n}] = ISAC.singleBSSimulation(simulationTime, BSs(n), tops);
        end
    end

end
