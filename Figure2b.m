%Energy-Efficient Data Collection in UAV Enabled Wireless Sensor Network Cheng Zhan , Member, IEEE, Yong Zeng , Member, IEEE, and Rui Zhang , Fellow, IEEE
% Jongseok Lee, kwangwoon university, seoul, korea.
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

%% parameter setting
parameter_setting('T', 100,'Sk',10*10^6);

epsilons = -1 : - 2/19:-3;
epsilons = 10.^epsilons;
z = 0:0.00001:1;
F = 1 - marcumq((2*Kc)^0.5, (2*(Kc+1)*z(:)).^0.5);

%% figure 2. (b) 

%% Static collection
figure()
Q_Static = mean(u,2);

theta = zeros(20,1);
for i = 1:20
    epsilon = epsilons(i);
    F_1 = z(find(F>epsilon,1,'first'));
    
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
semilogx(epsilons,theta,'-+k');

%% Straight flight
Q_Straight = Q0 ;
theta = zeros(20,1);
for i = 1: 20
    epsilon = epsilons(i);
   F_1 = z(find(F>epsilon,1,'first'));
    [tmp, energy] = solveP2(Q_Straight);
    theta(i) = energy;
end
hold on
semilogx(epsilons,theta,'-ob');

%% Optimized tajectory
Q_Opt = Fig1.Q{3};
theta = zeros(20,1);
for i = 1: 20
    epsilon = epsilons(i);
   F_1 = z(find(F>epsilon,1,'first'));
    [tmp, energy] = solveP2(Q_Opt);
    theta(i) = energy;
end
hold on
semilogx(epsilons,theta,'-xr');

%% Lower bound
thetalb =  zeros(20,1);
for i = 1: 20
    epsilon = epsilons(i);
    F_1 = z(find(F>epsilon,1,'first'));
    d = H;
    pathloss = getPathLoss(d);
    Rmax = getAchievableRate(pathloss);
   
    thetalb(i) = Pk * (Sk/ (B * Rmax));
end
hold on
semilogx(epsilons,thetalb,'->m');

%% Draw figure
title('Fig. 2.(b), \theta versus \epsilon ( \it S_{k} = 10 Mbits )','fontsize',15)
xlabel('Outage probability target \epsilon ','fontsize',15)
ylabel('min-max energy consumption (Joule)','fontsize',15)
xlim([0.001 0.1])
ylim([0 2.5])
set(gca, 'XDir','reverse')
set(gca, 'XScale', 'log')
grid on

legend('Static collecting','Stratight flight','Optimized trjectory','Lower bound','Location','NorthWest','fontsize',12);
