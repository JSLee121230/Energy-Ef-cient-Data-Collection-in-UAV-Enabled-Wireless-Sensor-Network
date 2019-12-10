%% getAchievableRate
% input, pathloss is dB scale
% output, R's unit is bps/hz
function [R] = getAchievableRate(pathloss)

global F_1 Pk sigma_2 Lamda

R = log2(1+F_1*Pk*dB2dec(pathloss)/(dB2dec(sigma_2)*dB2dec(Lamda)));

end

%% local functions
function [dB] = dec2dB(dec)
dB = 10*log10(dec);
end

function [dec] = dB2dec(dB)
dec = 10^(dB/10);
end
