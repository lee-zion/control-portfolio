%% Modern Control Ackermann design
clear all;clc;
w_m = 3.81*2*pi;                  % [rad/s]
tau_m       = 1/w_m;          % [s]
m_ball      = 0.11;         % [kg]
m_beam      = 0.16;         % [kg]
m_gear      = 0.110;        % [kg]
g           = 9.81;         % [m/s]
R_gear      = 0.03;
L_beam      = 0.4;
Jb          = 1/12*m_beam*L_beam^2+1/2*m_gear*R_gear^2;    % [kg*m^2]
A           = [0 1 0 0; 0 -1/tau_m -m_ball*g/Jb 0;0 0 0 1;5*g/9 0 0 0];
B           = [0;1/tau_m;0;0];
% C           = [1 0 0 0;0 0 1 0];
C           = [0 0 1 0];

I           = eye(4);

a           = charpoly(A);
CO          = ctrb(A,B);
WO          = obsv(A,C);
W           = [a(4) a(3) a(2) 1;   
               a(3) a(2) 1    0;
               a(2) 1    0    0;
               1    0    0    0];
           
T           = CO*W ;
A_          = inv(T)*A*T;
B_          = inv(T)*B;
CO_         = ctrb(A_,B_);
controb     = rank(CO_);
observb     = rank(W);

%%
cmd = 0;
sample_time = 0.005;
final_time = 20;
w_norm  = (0.2:0.01:1.0) ;
Legend  = '';
color   = {[1 0 0];[0 0 1];[0 1 0];[0 0 0];[0 1 1];[1 0 1];[0.2 0.3 0.8];[0.9 0.2 0.1]}; % 8
cmd     = 0;
i= 1;
for zeta_dom = [0.4 0.5 0.6 0.7 0.8]
    K       = [];
    tr      = [];
    ts      = [];
    OS      = [];
    Pm      = [];
    Gm      = [];
    Pm      = [];
    P       = [];
    for omega_dom     = w_norm * w_m
        omega_d     = omega_dom*sqrt(1-zeta_dom^2);
        p1          = -zeta_dom*omega_dom+omega_d*j; 
        p2          = -zeta_dom*omega_dom-omega_d*j; 
        p3          = -zeta_dom*omega_dom*5;
        p4          = -zeta_dom*omega_dom*5;
        P_t         = [p1 p2 p3 p4];
        P           = [P; P_t];
        K           = [K; acker(A,B,P_t)];
        Acl         = A - B*K(end,:) ;
%         [V, D]      = eig(Acl) ;
        % disp(inv(V)*Acl*V);
        N_bar       = (-C*inv(A-B*K(end,:))*B).^-1;

        syscl       = ss(Acl,B*N_bar,C,0);

        % Break Point
        [Ac, Bc, Cc, Dc] = linmod('ModernControlSimOpenLoop') ; %test margin chek
        sys = ss(Ac, Bc, Cc, Dc) ;
        Go  = -tf(sys)    ; 
        Go  = minreal(Go) ; % because of many numerical error 
        [ Gm_t, Pm_t, Wpc, Wgc ] = margin(Go) ;
        Gm      = [Gm 20*log10(Gm_t)];
        Pm      = [Pm Pm_t];
        
        s       = stepinfo(syscl,'RiseTimeLimits',[0.0,0.9]);
        tr      = [tr s.RiseTime];
        ts      = [ts s.SettlingTime];
        OS      = [OS (s.Overshoot)];
   end
        Legend = [Legend ; strcat('\it \zeta_c = ',sprintf('%4.3f',zeta_dom(end)))];
        figure(1);
        subplot(2,4,1);
        grid on;hold on; plot(w_norm,tr,'color',cell2mat(color(i)),'linewidth',2); title('Risetime','fontsize',14); ylabel('[sec]'); xlabel('\it \omega_c/\omega_n'); ylim([0 1]);legend( Legend ); 
        subplot(2,4,2);
        grid on; hold on; plot(w_norm,OS,'color',cell2mat(color(i)),'linewidth',2); title('% Overshoot','fontsize',14);ylabel('[%]'); xlabel('\it \omega_c/\omega_n');  ylim([0 100]);legend( Legend );
        subplot(2,4,3);
        grid on; hold on; plot(w_norm,Gm,'color',cell2mat(color(i)),'linewidth',2); title('Gain margin','fontsize',14);ylabel('[dB]'); xlabel('\it \omega_c/\omega_n'); legend( Legend );
        subplot(2,4,4);
        grid on; hold on;plot(w_norm,Pm,'color',cell2mat(color(i)),'linewidth',2); title('Phase margin','fontsize',14);ylabel('[deg]'); xlabel('\it \omega_c/\omega_n'); legend( Legend ); 
        subplot(2,4,5);
        grid on; hold on; plot(w_norm,K(:,1),'color',cell2mat(color(i)),'linewidth',2); title('K1','fontsize',14); xlabel('\it \omega_c/\omega_n'); legend( Legend );
        subplot(2,4,6);
        grid on; hold on; plot(w_norm,K(:,2),'color',cell2mat(color(i)),'linewidth',2); title('K2','fontsize',14); xlabel('\it \omega_c/\omega_n'); legend( Legend );
        subplot(2,4,7);
        grid on; hold on; plot(w_norm,K(:,3),'color',cell2mat(color(i)),'linewidth',2); title('K3','fontsize',14); xlabel('\it \omega_c/\omega_n'); legend( Legend );
        subplot(2,4,8);
        grid on; hold on; plot(w_norm,K(:,4),'color',cell2mat(color(i)),'linewidth',2); title('K4','fontsize',14); xlabel('\it \omega_c/\omega_n'); legend( Legend );
        figure(2);
        subplot(2,1,1);
        hold on; plot(w_norm,P(:,3),'color',cell2mat(color(i)),'linewidth',2);title('p3','fontsize',14);legend( Legend );
        subplot(2,1,2);
        hold on; plot(w_norm,P(:,4),'color',cell2mat(color(i)),'linewidth',2);title('p4','fontsize',14);legend( Legend );
        i = i+1;
end
%% Desired Poles

zeta_dom = 0.591155;
omega_dom = 6.766415;

tr = (2.16*zeta_dom + 0.6)/omega_dom;
ts = 4/(zeta_dom*omega_dom);
OS = exp(-pi*zeta_dom/sqrt(1-zeta_dom^2))*100;
 omega_d     = omega_dom*sqrt(1-zeta_dom^2);
        p1          = -zeta_dom*omega_dom+omega_d*j; 
        p2          = -zeta_dom*omega_dom-omega_d*j; 
        p3          = -zeta_dom*omega_dom*5;
        p4          = -zeta_dom*omega_dom*5;
        P_t         = [p1 p2 p3 p4];
        K           = acker(A,B,P_t);
        Acl         = A - B*K(end,:) ;
%         [V, D]      = eig(Acl) ;
        % disp(inv(V)*Acl*V);
        N_bar       = (-C*inv(A-B*K(end,:))*B).^-1;
%% Observer
ppfactor = 3;
obs_zeta_dom = 0.4;
obs_omega_dom = omega_dom*ppfactor;
lp1 = -obs_zeta_dom*obs_omega_dom; 
lp2 = -obs_zeta_dom*obs_omega_dom; 
lp3 = -obs_zeta_dom*obs_omega_dom;
lp4 = -obs_zeta_dom*obs_omega_dom;

LP = [lp1 lp2 lp3 lp4];
L = acker(A_',C',LP);
L = L';
% L = [1 5;2 6;3 7;4 8];

%% Plot
close all; clc;
final_time = 30;
sample_time = 0.005;
cmd = 10;
sim('ModernControlSim');
figure();
plot(simtime,simout(:,1),'-r','linewidth',2), % theta
hold on; 
plot(simtime,simout(:,2),'-k','linewidth',2); % theta_dot
plot(simtime,simout(:,3),'g','linewidth',2); % x
plot(simtime,simout(:,4),'-b','linewidth',2); % x_dot
plot(simtime,simout(:,5),'--m','linewidth',2); % x_measured
% plot(simtime,simout(:,6),'c'); 
grid on
xlim([0 5]);
legend('(x1) \theta[rad]','(x2) \theta-dot[rad/s]','(x3) x[m]','(x4) x-dot[m/s]','(y) x[m]');
title(strcat('Ackermann Method Simulation, Observer Pole = \omega_c_l *',sprintf(' %2.1f ',ppfactor)),'fontsize',14)
%%
%%
clear all ; close all ; clc

%%
T_Start = 0;
T_Final = 10;
sample_time = 1/200;
%%
Km          = 1.2204 ;
Wm         = 3.81*2*pi;                  % [rad/s]
tau_m       = 1/Wm;          % [s]
m_ball      = 0.11;         % [kg]
m_beam      = 0.16;         % [kg]
m_gear      = 0.110;        % [kg]
g           = 9.81;         % [m/s]
R_gear      = 0.03;
L_beam      = 0.4;
Jb          = 1/12*m_beam*L_beam^2+1/2*m_gear*R_gear^2;    % [kg*m^2]
a1          = -1/tau_m;
a2          = m_ball*g/Jb;
a3          = 5*g/9;
b1          = Km*Wm;

A           = [0  1  0  0
               0  a1 a2 0
               0  0  0  1
               a3 0  0  0];
B           = [0
               b1
               0
               0];

C           = [0 0 1 0];

I           = eye(4);

a       = charpoly(A);
CO      = ctrb(A,B);
WO      = obsv(A,C);
W       = [a(4) a(3) a(2) 1;   
               a(3) a(2) 1    0;
               a(2) 1    0    0;
               1    0    0    0];
           
T       = CO*W ;
A_      = inv(T)*A*T;
B_      = inv(T)*B;
CO_     = ctrb(A_,B_);
controb = rank(CO_);
observb = rank(W);

CMD = -0.05 ; 
x_0 = 0.05 ; 
NdB = 1e-5 ; 
K = [26.45 0.8325 131.0 31.78] ; 
N_bar = (b1*K(3)-a2)/b1;
% N_bar = 1 ; 

sim('ModernControlSim.mdl')

%
data = load('FMF.txt') ; 
time = data(:, 1) ; 
cmd = data(:, 2) ; 
fA = data(:, 3) ; 
fW = data(:, 4) ; 
fX = data(:, 5) ; 
fV = data(:, 6) ; 
mX = data(:, 7) ; 
mAng = data(:, 8) ; 

% data2 = load('OBS.txt') ; 
% eA = data2(:, 3) ; 
% eW = data2(:, 4) ; 
% eX = data2(:, 5) ; 
% eV = data2(:, 6) ; 

SIM_A = simout(:, 1) ; 
SIM_W = simout(:, 2) ; 
SIM_X = simout(:, 3) ; 
SIM_V = simout(:, 4) ; 

simtime = simtime+2 ; 

figure('color', 'white')
xlabel('Time [sec]'), ylabel('Postion [m]')
plot(time, cmd, '*k', simtime, SIM_X, '--b', time, fX, '-m')
legend('CMD', 'SIM_X', 'fX')

% figure('color', 'white')
% xlabel('Time [sec]'), ylabel('Postion [m]')
% plot(time, cmd, '*k', time, mX, '--b', time, fX, '-m')
% legend('CMD', 'mX', 'fX')