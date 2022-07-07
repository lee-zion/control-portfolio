%%
clear all
clc
% 1. Data index generate
tblFreq = linspace(4/5, 4/3, 10);
Nfreq   = length(tblFreq);
 
% 2. Motor response LS curve fitting
for idxFreq = 1:Nfreq;
    
    freq = tblFreq(idxFreq);
    w(idxFreq)=freq*2*pi;
   
    % 1) Motor I/O data loading
    
    filename = strcat(sprintf('%d',idxFreq-1),'.out.txt');
    data = load(filename);
    
    Time    = data(100:1900,1);
    Wcmd    = data(100:1900,2);
    Wfv     = data(100:1900,3);
    
    % 2) Motor response curve model
    SysResp = inline('Vfit(1)*sin( 2*pi*fTime + Vfit(2) )+Vfit(3)', ...
                     'Vfit', 'fTime');
    
    Wmag0  = 300 ; % initial magnitude guess
    Wphs0  = 0; % initial phase delay guess
    Wbias0 = 0; % initial bias guess
    
    % 3) LS estimated signal parameters extraction
    Wfit = lsqcurvefit(SysResp,[Wmag0;Wphs0;Wbias0],freq*Time,Wfv );
        
    eMag(idxFreq)  = Wfit(1) ; % estimated magnitude
    ePhs(idxFreq)  = Wfit(2) ; % estimated phase delay, Radian !!!!!!
    eBias(idxFreq) = Wfit(3) ; % estimated bias 
    
    %4) Estimated motor response recomposition
    eWfv = eMag(idxFreq)*sin( 2*pi*freq*Time+ePhs(idxFreq) )+eBias(idxFreq);
    
% %     5) Plot original and fitted response
%     figure(idxFreq);
%     plot(Time, Wcmd, 'k', Time, Wfv, 'r--', Time, eWfv, 'b');
%     grid on, xlabel('time [sec]'), ylabel('Vcmd and Vfv [V]');
%     legend('measured Vcmd', 'measured Vfv', 'fitted Vfv', 0) ;
%     
%     strTitle = sprintf( 'Freq = %2.1f[Hz] : Gain = %8.4f, Phase Shift = %8.4f[deg]', ...
%                        freq, eMag(idxFreq), ePhs(idxFreq)*180/(pi));
%     title(strTitle) ;
end

% 3. Product motor transfer function
Nnum = 0;
Nden = 2;
tblOmega      = 2*pi*tblFreq     ; % Unit conversion [Hz -> rad/s]
MeasFreqResp = eMag./Wmag0.* exp(i*(ePhs));
[num, den]   = invfreqs( MeasFreqResp, tblOmega, Nnum, Nden ) ;
Gm = tf( num, den ) ;
[num_m, den_m] = tfdata(Gm, 'v');
Wm = (den_m(3))^0.5 ;
Zm = (den_m(2)) / 2 / Wm ;
Km = (num_m(3))/den_m(3) ;
%%    
    % 4. Plot bode-plot
figure,  plot(w,20*log10(eMag/Wmag0),'rx'), hold on
bode( Gm ), hold on 
plot(w,ePhs*180/pi,'rx') , hold on
