%% 2014 자동제어 볼앤빔 프로젝트 !
 %% PID controller for Ball and beam system
clc;
clear all;
close all;
 
Ts = 0.005;
Tstart = 0;
Tfinal = 1;
s = tf('s') ;
%actuator model transfer function

num_motor = [0 0 790.3] ;
den_motor = [1 39.72 805.6] ;


Wm = den_motor(3)^0.5 ;
Zm = den_motor(2) / 2 / Wm ;
Km = num_motor(3) / den_motor(3) ;
Gmotor = Km*Wm^2 / (s^2 + 2*Zm*Wm*s + Wm^2);
% Omega control design
Z_omega = 0.707 ;
W_omega = 24/ Z_omega;
alpha = 0.4 ;
r_omega = Z_omega * W_omega *alpha ; 
tau = 4*Z_omega*W_omega;


Kd_omega = ( 2*Z_omega*W_omega + r_omega - 2*Zm*Wm ) / (Km*Wm^2) ;
Kp_omega = ( 2*Z_omega*W_omega*r_omega + W_omega^2 - Wm^2) / (Km*Wm^2) ;
Ki_omega = r_omega*W_omega^2 / (Km*Wm^2) ;


Gc_omega = Kp_omega + Kd_omega*s + Ki_omega/s;
Gcl_omega = Gmotor*Gc_omega/(1+Gmotor*Gc_omega) ;
Gcl_omega = minreal(Gcl_omega) ;

% Angle control design
Tm = bandwidth(Gcl_omega) ;
Gapp = Tm / (s+Tm) ;
Z_angle = 0.90;
W_angle = 16/Z_angle;
Kw_angle = (2*Z_angle*W_angle - Tm ) / Tm ;
Kt_angle = W_angle^2 / Tm ;

G_inner = minreal(  Gcl_omega / (1+Gcl_omega*Kw_angle) * Kt_angle/s );
Gcl_angle = minreal ( G_inner / (1+G_inner) );


% position control design



%% margin check

% omega control loop
[Ao,Bo,Co,Do] = linmod('test_margin_Omega_control_loop'); % Modeling minear model
sysGomega = -ss(Ao,Bo,Co,Do); % Construct state-space model or convert model to state space
sysGomega = minreal(sysGomega); % Eliminate errors
Gomega_margin = tf(sysGomega); % Making Transferfunction

[GM_omega, PM_omega, Wpc_omega, Wgc_omega] = margin(Gomega_margin);


% angle control loop
[Aa,Ba,Ca,Da] = linmod('test_margin_Angle_control_loop'); % Modeling minear model
sysGangle = -ss(Aa,Ba,Ca,Da); % Construct state-space model or convert model to state space
sysGangle = minreal(sysGangle); % Eliminate errors
Gangle_margin = tf(sysGangle); % Making Transferfunction

[GM_angle, PM_angle, Wpc_angle, Wgc_angle] = margin(Gangle_margin);

%%
% position control loop
[Ap,Bp,Cp,Dp] = linmod('test_margin_Position_Control_Loop'); % Modeling minear model
sysGposition = -ss(Ap,Bp,Cp,Dp); % Construct state-space model or convert model to state space
sysGposition = minreal(sysGposition); % Eliminate errors
Gposition_margin = tf(sysGposition) ; % Making Transferfunction

[GM_position, PM_position, Wpc_position, Wgc_position] = margin(Gposition_margin);





%% simulation
sim('Omega_control_loop') ;
sim('Angle_Control_Loop') ;
%sim('Position_Control_Loop') ;
%% plots
figure, pzmap(Gcl_omega) ;
figure, nyquist(Gomega_margin);
figure,  plot(Time, Omega_c, Time, Omega_o);

figure, pzmap(Gcl_angle) ;
figure,  plot(Time, Angle_c, Time, Angle_o);
figure,  nyquist(Gangle_margin);






