function [pathloss] = getPathLoss(d)
% input, d is decimal scale
% output pathloss is dB scale
    global beta0 alpha

    da = -alpha*dec2dB(d);
    pathloss = beta0 + da;
end

%% local functions
function [dB] = dec2dB(dec)
dB = 10*log10(dec);
end

function [dec] = dB2dec(dB)
dec = 10^(dB/10);
end
