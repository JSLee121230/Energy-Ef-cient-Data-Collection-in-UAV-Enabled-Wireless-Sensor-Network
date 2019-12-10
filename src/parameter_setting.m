function parameter_setting(varargin)
global K M H w alpha beta0 Vmax delta_t Dmax B sigma_2 Lamda Kc Pk q0 qF Sk epsilon T F_1 Ek rk u tolerance Q0
if nargin==0
    Sk = 10*10^6; %Mbits
    T = 50;
    epsilon = 10^-2;
else
    for i = 1 : nargin
        if strcmp(varargin{i}, 'Sk')
            Sk = varargin{i+1};
            break;
        else
            Sk = 10*10^6; %Mbits
        end
    end
    for i = 1 : nargin
        if strcmp(varargin{i}, 'T')
            T = varargin{i+1};
            break;
        else
            T = 50;
        end
    end
    for i = 1 : nargin
        if strcmp(varargin{i}, 'epsilon')
            epsilon = varargin{i+1};
            break;
        else
            epsilon = 10^-2;
        end
    end
end
K = 4; % SNs
q0 = [-800, 0]'; % m
qF = [ 800, 0]'; % m
H = 100;       % m
Vmax = 50; % m/s
delta_t = 0.5; % s
Dmax = Vmax*delta_t;
B = 1*10^6; %MHz
beta0    =   -60;   % %10^(-6);  %dB <- -60dB
sigma_2 = -140;  %10^(-14); %dB <- -110dBm
Lamda   =    7 ; %10^(0.7); %dB <- 7dB
Kc = 10;
alpha = 2;
Pk = 0.1; %W
rk = Sk/(B*delta_t); % 10Mbit/(1Mhz*0.5s)

u1 = [-600 ;  400];
u2 = [-300 ; -500];
u3 = [ 200 ;  500];
u4 = [ 600 ; -400];

u = [u1,u2,u3,u4];
M = (T/ delta_t);
m = 1 : 1: M;
Q0 = ((qF - q0)*(m-1)/(M-1)) + q0;

z = 0:0.001:1;
F = 1 - marcumq((2*Kc)^0.5, (2*(Kc+1)*z(:)).^0.5,1);
F_1 = z(find(F>=epsilon,1,'first'));

w = zeros(2, M, K);
for k = 1: K
    w(:,:,k) = repelem( u(: ,k),1,M);
end
Ek = Pk*delta_t;
tolerance = 0.000001; %0.00000001; % user parameter

end
