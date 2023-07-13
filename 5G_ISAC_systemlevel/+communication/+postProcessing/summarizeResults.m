function results = summarizeResults(dataState)
% Summarize simulation results
    
    numUEs = numel(dataState);
    BLER = zeros(numUEs,1);
    Throughput = zeros(numUEs,1);
    
    for ue = 1:numUEs
        
        CRC = dataState(ue).CRC;
        CRC = CRC(~isnan(CRC));
        BLER(ue) = sum(CRC) / numel(CRC);
        Throughput(ue) = sum(dataState(ue).Throughput);
        
    end
    
    User = (1:numUEs).';
    results = table(User,BLER,Throughput);
    results.Properties.VariableNames{3} = 'Throughput (bits)';
    
end

