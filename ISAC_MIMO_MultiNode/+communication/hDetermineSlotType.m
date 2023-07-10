function [slotType,nSymbolsDL] = hDetermineSlotType(tddPattern,specialSlot,carrier)
%HDETERMINESLOTTYPE 
%   
    % Determine the number of downlink OFDM symbols
    slotType = tddPattern(mod(carrier.NSlot,numel(tddPattern))+1);
    if (slotType == "D")
        nSymbolsDL = carrier.SymbolsPerSlot;
    elseif (slotType == "S")
        nSymbolsDL = specialSlot(1);
    else % slotType=="U"
        nSymbolsDL = 0;
    end
end

