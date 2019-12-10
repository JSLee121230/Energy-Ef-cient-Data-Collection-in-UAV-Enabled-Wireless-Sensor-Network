function [Xr, Theta] = solveP2(Qr)
global M K Ek rk

% Equation (1)
pathloss = eq1(Qr);

% Equation (4)
R = eq4(pathloss);

cvx_begin quiet  %(P2)
variables x(M,K);
variables theta(1,1);

minimize(sum(theta))
subject to:
0 <= x <= 1;              % Constraint(11)
sum(x.*Ek, 1) <= theta  % Constraint(5)
sum(x.*R, 1) >= rk       % Constraint(6)
sum(x    , 2) <= 1       % Constraint(7)
cvx_end

Xr = x;

Theta = sum(theta);

end

function [pathloss] = eq1(Qr)
global K M H w  

Qr  = repelem(Qr,1,1,K);                        % 거리 계산을 쉽게 하기 위하여 3차원 matrix로 복사 확장
d = (H^2 + sum((Qr - w).^2 ,1)).^(0.5);
pathloss = getPathLoss(d);                     % eq1  계산
pathloss = reshape(pathloss, [M,K] );        % 출력을 위해 matrix 재정렬
end

function [R] = eq4(pathloss)
R = getAchievableRate(pathloss);
end

function [dB] = dec2dB(dec)
dB = 10*log10(dec);
end

function [dec] = dB2dec(dB)
dec = 10.^(dB/10);
end


function [R] = getAchievableRate(pathloss)
% input, pathloss is dB scale
% output, R's unit is bps/hz
global F_1 Pk sigma_2 Lamda
R = log2(1+(F_1*Pk*dB2dec(pathloss)) / (dB2dec(sigma_2)*dB2dec(Lamda)));
end

function [pathloss] = getPathLoss(d)
% input, d is decimal scale
% output pathloss is dB scale
    global beta0 alpha

    da = -alpha*dec2dB(d);
    pathloss = beta0 + da;
end
