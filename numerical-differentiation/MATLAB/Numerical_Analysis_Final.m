%% Numerical analysis Final project
% Brief comments for the source code
% 

% clear all
% close all
% clc

Tstart = 0.0;
Tfinal = 2.0;
Ts = 0.0005;

Kp = 27;
Kd = 0.003;
zetaA = 0.7;
zetaS = 0.7;
Ws = 2*pi*100;

% [rad/s]
Was = [2*pi*16.4, 2*pi*20.5, 2*pi*24.6];

phi_cs = [0, pi/8, pi/4];

Lp = 3.2;
Lphi = 1200;
Ldelta = 16000;

% Ga = tf([Wa*Wa], [1 2*zetaA*Wa Wa*Wa]);
% Gs = tf([Ws*Ws], [1 2*zetaS*Ws Ws*Ws]);

for i = 2:3

    for j = 1:3
        Wa = Was(i);
        phi_c = phi_cs(j);
        phi_initial = phi_c + pi/180;
        delta_initial = phi_c * Kp * Kd;
        
        % Initial condition of initial roll angle, 1[rad] ,about 55[deg]
        
        sim('Numerical_Analysis_Final_Model')
        
        if j == 1
            
            figure('Color', 'White')
            plot(time, phi_c, 'r', time, phi, 'g'), xlabel('Time [sec]'), ylabel('\phi = 0[deg]'), legend('\phi_c', '\phi')
            
            if i == 1
                title('\omega_a = 16.4[Hz]')
            end
            if i == 2
                title('\omega_a = 20.5[Hz]')
            end
            if i == 3
                title('\omega_a = 24.6[Hz]')
            end
            
            figure('Color', 'White')
            plot(time, p), xlabel('Time [sec]'), ylabel('p [ deg/s ]'), legend('p')
            
            if i == 1
                title('\omega_a = 16.4[Hz]')
            end
            if i == 2
                title('\omega_a = 20.5[Hz]')
            end
            if i == 3
                title('\omega_a = 24.6[Hz]')
            end
            
            figure('Color', 'White')
            plot(time, deltac, 'r', time, delta, 'g'), xlabel('Time [sec]'), ylabel('\delta [deg]'), legend('\delta_c', '\delta')
            
            if i == 1
                title('\omega_a = 16.4[Hz]')
            end
            if i == 2
                title('\omega_a = 20.5[Hz]')
            end
            if i == 3
                title('\omega_a = 24.6[Hz]')
            end
            
        end
        
        if j == 2
            figure('Color', 'White')
            plot(time, phi_c, 'r', time, phi, 'g'), xlabel('Time [sec]'), ylabel('\phi = \pi/8[deg]'), legend('\phi_c', '\phi')
            
            if i == 1
                title('\omega_a = 16.4[Hz]')
            end
            if i == 2
                title('\omega_a = 20.5[Hz]')
            end
            if i == 3
                title('\omega_a = 24.6[Hz]')
            end
            
            
            figure('Color', 'White')
            plot(time, p), xlabel('Time [sec]'), ylabel('p [ deg/s ]'), legend('p')

            if i == 1
                title('\omega_a = 16.4[Hz]')
            end
            if i == 2
                title('\omega_a = 20.5[Hz]')
            end
            if i == 3
                title('\omega_a = 24.6[Hz]')
            end
            
            
            figure('Color', 'White')
            plot(time, deltac, 'r', time, delta, 'g'), xlabel('Time [sec]'), ylabel('\delta [deg]'), legend('\delta_c', '\delta')

            if i == 1
                title('\omega_a = 16.4[Hz]')
            end
            if i == 2
                title('\omega_a = 20.5[Hz]')
            end
            if i == 3
                title('\omega_a = 24.6[Hz]')
            end
        
        end
        
        if j == 3
            figure('Color', 'White')
            plot(time, phi_c, 'r', time, phi, 'g'), xlabel('Time [sec]'), ylabel('\phi = \pi/4[deg]'), legend('\phi_c', '\phi')

            if i == 1
                title('\omega_a = 16.4[Hz]')
            end
            if i == 2
                title('\omega_a = 20.5[Hz]')
            end
            if i == 3
                title('\omega_a = 24.6[Hz]')
            end
            
            
            figure('Color', 'White')
            plot(time, p), xlabel('Time [sec]'), ylabel('p [ deg/s ]'), legend('p')

            if i == 1
                title('\omega_a = 16.4[Hz]')
            end
            if i == 2
                title('\omega_a = 20.5[Hz]')
            end
            if i == 3
                title('\omega_a = 24.6[Hz]')
            end
            
            
            figure('Color', 'White')
            plot(time, deltac, 'r', time, delta, 'g'), xlabel('Time [sec]'), ylabel('\delta [deg]'), legend('\delta_c', '\delta')

            if i == 1
                title('\omega_a = 16.4[Hz]')
            end
            if i == 2
                title('\omega_a = 20.5[Hz]')
            end
            if i == 3
                title('\omega_a = 24.6[Hz]')
            end
        
        end        
    end   
end



%         k = j;
%         subplot(3,3,k)      
%                                                                                                         if j == 1
%                                                                                                             title('\psi_c = 0 [deg]')
%                                                                                                         end
% 
%                                                                                                         if j == 2
%                                                                                                             title('\psi_c = \pi/8 [deg]')
%                                                                                                         end
% 
%                                                                                                         if j == 3
%                                                                                                             title('\psi_c = \pi/4 [deg]')
%                                                                                                         end    
%         plot(time, phi_c*180/pi, 'r', time, phi*180/pi, 'g'), xlabel('Time [sec]'), ylabel('\phi [deg]')
%         
%         k = k+3;
%         subplot(3,3,k)
%                                                                                                         if j == 1
%                                                                                                             title('\psi_c = 0 [deg]')
%                                                                                                         end
% 
%                                                                                                         if j == 2
%                                                                                                             title('\psi_c = \pi/8 [deg]')
%                                                                                                         end
% 
%                                                                                                         if j == 3
%                                                                                                             title('\psi_c = \pi/4 [deg]')
%                                                                                                         end
%         plot(time, p*180/pi), xlabel('Time [sec]'), ylabel('p [ deg/s ]')
%         
%         k = k+3;
%         subplot(3,3,k)
%         if j == 1
%             title('\psi_c = 0 [deg]')
%         end
% 
%         if j == 2
%             title('\psi_c = \pi/8 [deg]')
%         end
% 
%         if j == 3
%             title('\psi_c = \pi/4 [deg]')
%         end
% 
%         plot(time, deltac*180/pi, 'r', time, delta*180/pi, 'g'), xlabel('Time [sec]'), ylabel('\delta [deg]')


%% Plot 3 figures at C program
% time vs phic & phi
% time vs p
% time vs deltac & delta

clear all
close all
clc

Data1 = load('103.044239_  0.000000_.txt');
Time1 = Data1(:, 1); phic1 = Data1(:, 2); phi1 = Data1(:, 3);
p1 = Data1(:, 4); deltac1 = Data1(:, 5); delta1 = Data1(:, 6);

figure('Color', 'White')
plot(Time1, phic1, 'r', Time1, phi1, 'g'), xlabel('Time [sec]'), ylabel('\phi = 0[deg]'), legend('\phi_c', '\phi')

figure('Color', 'White')
plot(Time1, p1), xlabel('Time [sec]'), ylabel('p [ deg/s ]'), legend('p')

figure('Color', 'White')
plot(Time1, deltac1, 'r', Time1, delta1, 'g'), xlabel('Time [sec]'), ylabel('\delta [deg]'), legend('\delta_c', '\delta')




Data2 = load('103.044239_  0.392699_.txt');
Time2 = Data2(:, 1); phic2 = Data2(:, 2); phi2 = Data2(:, 3);
p2 = Data2(:, 4); deltac2 = Data2(:, 5); delta2 = Data2(:, 6);

figure('Color', 'White')
plot(Time2, phic2, 'r', Time2, phi2, 'g'), xlabel('Time [sec]'), ylabel('\phi = 0[deg]'), legend('\phi_c', '\phi')

figure('Color', 'White')
plot(Time2, p2), xlabel('Time [sec]'), ylabel('p [ deg/s ]'), legend('p')

figure('Color', 'White')
plot(Time2, deltac2, 'r', Time2, delta2, 'g'), xlabel('Time [sec]'), ylabel('\delta [deg]'), legend('\delta_c', '\delta')




Data3 = load('103.044239_  0.785398_.txt');
Time3 = Data3(:, 1); phic3 = Data3(:, 2); phi3 = Data3(:, 3);
p3 = Data3(:, 4); deltac3 = Data3(:, 5); delta3 = Data3(:, 6);

figure('Color', 'White')
plot(Time3, phic3, 'r', Time3, phi3, 'g'), xlabel('Time [sec]'), ylabel('\phi = 0[deg]'), legend('\phi_c', '\phi')

figure('Color', 'White')
plot(Time3, p3), xlabel('Time [sec]'), ylabel('p [ deg/s ]'), legend('p')

figure('Color', 'White')
plot(Time3, deltac3, 'r', Time3, delta3, 'g'), xlabel('Time [sec]'), ylabel('\delta [deg]'), legend('\delta_c', '\delta')




Data4 = load('128.805299_  0.000000_.txt');
Time4 = Data4(:, 1); phic4 = Data4(:, 2); phi4 = Data4(:, 3);
p4 = Data4(:, 4); deltac4 = Data4(:, 5); delta4 = Data4(:, 6);

figure('Color', 'White')
plot(Time4, phic4, 'r', Time4, phi4, 'g'), xlabel('Time [sec]'), ylabel('\phi = 0[deg]'), legend('\phi_c', '\phi')

figure('Color', 'White')
plot(Time4, p4), xlabel('Time [sec]'), ylabel('p [ deg/s ]'), legend('p')

figure('Color', 'White')
plot(Time4, deltac4, 'r', Time4, delta4, 'g'), xlabel('Time [sec]'), ylabel('\delta [deg]'), legend('\delta_c', '\delta')




Data5 = load('128.805299_  0.392699_.txt');
Time5 = Data5(:, 1); phic5 = Data5(:, 2); phi5 = Data5(:, 3);
p5 = Data5(:, 4); deltac5 = Data5(:, 5); delta5 = Data5(:, 6);

figure('Color', 'White')
plot(Time5, phic5, 'r', Time5, phi5, 'g'), xlabel('Time [sec]'), ylabel('\phi = 0[deg]'), legend('\phi_c', '\phi')

figure('Color', 'White')
plot(Time5, p5), xlabel('Time [sec]'), ylabel('p [ deg/s ]'), legend('p')

figure('Color', 'White')
plot(Time5, deltac5, 'r', Time5, delta5, 'g'), xlabel('Time [sec]'), ylabel('\delta [deg]'), legend('\delta_c', '\delta')




Data6 = load('128.805299_  0.785398_.txt');
Time6 = Data6(:, 1); phic6 = Data6(:, 2); phi6 = Data6(:, 3);
p6 = Data6(:, 4); deltac6 = Data6(:, 5); delta6 = Data6(:, 6);

figure('Color', 'White')
plot(Time6, phic6, 'r', Time6, phi6, 'g'), xlabel('Time [sec]'), ylabel('\phi = 0[deg]'), legend('\phi_c', '\phi')

figure('Color', 'White')
plot(Time6, p6), xlabel('Time [sec]'), ylabel('p [ deg/s ]'), legend('p')

figure('Color', 'White')
plot(Time6, deltac6, 'r', Time6, delta6, 'g'), xlabel('Time [sec]'), ylabel('\delta [deg]'), legend('\delta_c', '\delta')




Data7 = load('154.566359_  0.000000_.txt');
Time7 = Data7(:, 1); phic7 = Data7(:, 2); phi7 = Data7(:, 3);
p7 = Data7(:, 4); deltac7 = Data7(:, 5); delta7 = Data7(:, 6);

figure('Color', 'White')
plot(Time7, phic7, 'r', Time7, phi7, 'g'), xlabel('Time [sec]'), ylabel('\phi = 0[deg]'), legend('\phi_c', '\phi')

figure('Color', 'White')
plot(Time7, p7), xlabel('Time [sec]'), ylabel('p [ deg/s ]'), legend('p')

figure('Color', 'White')
plot(Time7, deltac7, 'r', Time7, delta7, 'g'), xlabel('Time [sec]'), ylabel('\delta [deg]'), legend('\delta_c', '\delta')




Data8 = load('154.566359_  0.392699_.txt');
Time8 = Data8(:, 1); phic8 = Data8(:, 2); phi8 = Data8(:, 3);
p8 = Data8(:, 4); deltac8 = Data8(:, 5); delta8 = Data8(:, 6);

figure('Color', 'White')
plot(Time8, phic8, 'r', Time8, phi8, 'g'), xlabel('Time [sec]'), ylabel('\phi = 0[deg]'), legend('\phi_c', '\phi')

figure('Color', 'White')
plot(Time8, p8), xlabel('Time [sec]'), ylabel('p [ deg/s ]'), legend('p')

figure('Color', 'White')
plot(Time8, deltac8, 'r', Time8, delta8, 'g'), xlabel('Time [sec]'), ylabel('\delta [deg]'), legend('\delta_c', '\delta')





Data9 = load('154.566359_  0.785398_.txt');
Time9 = Data9(:, 1); phic9 = Data9(:, 2); phi9 = Data9(:, 3);
p9 = Data9(:, 4); deltac9 = Data9(:, 5); delta9 = Data9(:, 6);

figure('Color', 'White')
plot(Time9, phic9, 'r', Time9, phi9, 'g'), xlabel('Time [sec]'), ylabel('\phi = 0[deg]'), legend('\phi_c', '\phi')

figure('Color', 'White')
plot(Time9, p9), xlabel('Time [sec]'), ylabel('p [ deg/s ]'), legend('p')

figure('Color', 'White')
plot(Time9, deltac9, 'r', Time9, delta9, 'g'), xlabel('Time [sec]'), ylabel('\delta [deg]'), legend('\delta_c', '\delta')


for i = 1 : 27
    saveas(figure(i), strcat(num2str(i), '_c'), 'jpg')
end

close all

