function plotResults(dataState, numFrames)
% Plot simulation results
    
    numUEs = numel(dataState);
    BLER = zeros(numUEs,1);
    Throughput = zeros(numUEs,1);
    
    for ue = 1:numUEs
        
        CRC = dataState(ue).CRC;
        CRC = CRC(~isnan(CRC));
        BLER(ue) = sum(CRC) / numel(CRC);
        Throughput(ue) = sum(dataState(ue).Throughput) / (numFrames * 0.01) / 1e6;
        
    end

    figure('Name', 'ECDF of DL Throughput')
    dlTp = tools.plotECDF(Throughput, 1);
    grid on
    legend(dlTp, 'Downlink throughput')
    title('ECDF of Downlink Throughput')
    xlabel('Data Rate (Mbps)')
    ylabel('Cumulative Probability')

    if ~any(isnan(BLER))
        figure('Name', 'ECDF of DL BLER')
        dlBler = tools.plotECDF(BLER, 1);
        grid on
        legend(dlBler, 'Downlink BLER')
        title('ECDF of Downlink BLER')
        xlim([0 1])
        xlabel('Block Error Rate')
        ylabel('Cumulative Probability')
    end
    
end

