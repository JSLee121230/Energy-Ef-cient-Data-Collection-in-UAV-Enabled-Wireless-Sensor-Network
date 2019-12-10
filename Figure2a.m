%Energy-Efficient Data Collection in UAV Enabled Wireless Sensor Network Cheng Zhan , Member, IEEE, Yong Zeng , Member, IEEE, and Rui Zhang , Fellow, IEEE
% Jongseok Lee, kwangwoon university, seoul, korea.
% Fig. 2(a) - energy consumption vs Sk

addpath('./src')
Fig1 = load('./data/Fig1.mat');
Xr = Fig1.X{3};
t = zeros(4,1);
for k = 1: 4
    t(k) = graythresh(Xr(:,k));
end
t = min(t);
Xr(Xr >= t) = 1;
Xr(Xr <   t) = 0;


global K M H w alpha beta0 Vmax delta_t Dmax B sigma_2 Lamda Kc Pk q0 qF Sk epsilon T F_1 Ek rk u tolerance Q0

parameter_setting('T', 100, 'epsilon',10^-2);
%% Parameters
% Location of SNs
u1 = [-600 ; 400];
u2 = [-300 ; -500];
u3 = [ 200 ;  500];
u4 = [ 600 ; -400];
u = [u1,u2,u3,u4];

% Initial point
q0 = [-800 ,0]';
% Final point
qF = [ 800 ,0]';

figure()
%% Static collection
Q_Static = mean(u,2);
theta = zeros(20,1);
for i = 1: 20
    Sk = i * 10^6; %
    x = sum(Xr);
    energy  = zeros(4,1);
    for k = 1 : 4               
        d = norm(Q_Static - u(:,k));
        pathloss = getPathLoss(d);
        Rk = getAchievableRate(pathloss);
        energy(k) =( Pk  * Sk / (B * Rk));
    end
    
    theta(i) =max(energy);
end
hold on
plot(theta,'-+k');

%% Straight flight
Q_Straight = Q0;
theta = zeros(20,1);
for i = 1: 20
    Sk = i * (10^6);
    rk = Sk/(B*delta_t); % 10Mbit/(1Mhz*0.5s)
    
    [tmp, energy] = solveP2(Q_Straight);
    theta(i) = energy;
end
hold on
plot(theta,'-ob');

%% Optimized trajectory
Q_Opt = Fig1.Q{3};
theta = zeros(20,1);
for i = 1: 20
    Sk = i * (10^6);
    rk = Sk/(B*delta_t); % 10Mbit/(1Mhz*0.5s)
    
    [tmp, energy] = solveP2(Q_Opt);
    theta(i) = energy;
end
hold on
plot(theta,'-xr');

%% Lower bound
thetalb =  zeros(20,1);
for i = 1: 20
    Sk = i * (10^6);
    d = H;
    pathloss = getPathLoss(d);
    Rmax = getAchievableRate(pathloss);
    
    thetalb(i) = Pk * (Sk/ (B * Rmax));
    
end
hold on
plot(thetalb,'->m');

%% Draw figure
title('Fig. 2.(a), \theta versus \it S_{k} ( \epsilon = 10^{-2})','fontsize',15)
xlabel('Data size S_{k} (Mbits)','fontsize',15)
ylabel('min-max energy consumption (Joule)','fontsize',15)

xlim([1 20])
ylim([0 2.5])
xticks([1 5 10 15 20])
grid on
legend('Static collecting','Stratight flight','Optimized trjectory','Lower bound','Location','NorthWest','fontsize',12);
