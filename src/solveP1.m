function [Q, X, Theta, Lo] = solveP1(varargin)
global u tolerance Q0 q0 K qF T
parameter_setting(varargin{:});
% close all

figure(T)
hold on
scatter(u(1,:),u(2,:), '*');
axis([ -800 800 -800 800])
grid on
title(['UAV tranjectory for each iteration (T=' num2str(T) 's)'])
% xlabel('x')
% ylabel('y')
text(q0(1),q0(2),'\leftarrow Initial point');
text(qF(1)-0.3,qF(2),'Final point \rightarrow');
for k = 1: K
    text(u(1,k),u(2,k),['u_{', num2str(k), '}']);
end
drawnow

%% Algorithm 2 Iterative Algorithm for Relaxed (P1)
% 1: AInitialize the trajectory as Q0;
Qr = Q0;

% 2: r ¡ç0; set tolerance ¥ê>0;
r = 0; tol = tolerance;

% 3: repeat
Th = [];
while 1
    % 4: Solve (P2) for given Qr to obtain solution Xr;
    [Xr, theta] = solveP2(Qr);
     figure(T+99); plot(Xr,'DisplayName','Xr'); title(['Wake-up schedule for each iteration  (T=' num2str(T) 's)'])
    
    % 5: Solve (P3) for given {Xr,Qr} with Algorithm 1, and denote the solution as Qr+1;
    [Qr, lolb]  = solveP3(Qr,Xr);
    
    % 6: r ¡çr+1;
    r  = r + 1;
    
    % print results
    disp(['r=',num2str(r),', theta: ', num2str(theta),', lolb: ',num2str(lolb)]);
    % display figures
    figure(T)
    hold on
    plot(Qr(1,:),Qr(2,:), '--');
    drawnow
    
    % 7:  until The fractional decrease of the objective value of (P2) is below ¥ê.
    Th = [Th ;theta];
    if (r >= 2) &&( Th(r-1) - Th(r) < tol)
        break;
    end
end
    % Returns
    Q = Qr;
    X = Xr;
    Theta = theta;
    Lo = lolb;
end

