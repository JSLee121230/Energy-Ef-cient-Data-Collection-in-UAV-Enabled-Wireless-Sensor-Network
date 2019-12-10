%% Algorithm 1. Successive Convex Optimization for (P3)
function [Qr, lolb] = solveP3(Qr,Xr)
global K M H alpha beta0 delta_t Dmax B sigma_2 Lamda Pk q0 qF Sk  F_1 w u tolerance

% 1: Initialize the trajectory as Q0;
Ql = Qr;

% 2: l¡ç0; set tolerance ¥ê>0;
l = 0; tol = tolerance;

Lo = [];
% 3: repeat
for l = 1: 1000
    
% 4:  Solve the QCQP problem (P4) for given Ql, and denote the optimal solution as Ql+1; 
    % Equation (16)
    Qlt  = repelem(Ql,1,1,K);
    J = H^2 + sum( (Qlt - w).^2 ,1); J = reshape(J, [M,K] );
    
    % Equation (15)
    I = F_1*Pk*dB2dec(beta0)*(alpha/2)*log2(exp(1)) ./ (J .* (dB2dec(sigma_2) * dB2dec(Lamda) * J .^(alpha/2) + F_1*Pk*dB2dec(beta0)));
    
    % Equation (14)
    A = log2(1 + (F_1*Pk*dB2dec(beta0)) ./ (dB2dec(sigma_2) * dB2dec(Lamda) * J .^(alpha/2)) );
    
    
cvx_begin quiet
    
    variables Q(2,M)
    variables lo(1)
    expression LO(K)
    
    maximize (sum(lo))
    
    subject to:
    % Constraint (17) and Equation (13)
    for k = 1 : K
        LO(k) = 0;
        for m = 1 : M
            Rlb = A(m,k) - I(m,k) * (norm( Q(:,m) - u(:,k))^2) + I(m,k) * (norm( Ql(: , m ) - u(:,k))^2);
            LO(k) = LO(k) + Xr(m,k) * Rlb;
        end
        (1/(Sk/(B*delta_t))) * LO(k) >= lo; %Constraint (17)
    end
    
    % constraint (9)
    for m = 2: M
       norm(Q(:,m) - Q(:,m-1)) <= Dmax;
    end
    
    % constraint (10)
    Q(1,1) == q0(1);
    Q(2,1) == q0(2);
    Q(1,M) == qF(1);
    Q(2,M) == qF(2);
    
cvx_end
    
    Lo = [Lo;sum(lo)];
    
% 5:  Ql ¡çQl+1; l¡çl+1;
    Ql = Q;

% 6: until The fractional increase of the objective value of (P4) is below ¥ê.

    if (l >= 2) &&(Lo(l) - Lo(l-1) < tol)
        break;
    end  
end

Qr = Q;
lolb = Lo(l);
end

function [dB] = dec2dB(dec)
dB = 10*log10(dec);
end

function [dec] = dB2dec(dB)
dec = 10.^(dB/10);
end