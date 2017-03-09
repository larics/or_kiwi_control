unix(['ssh root@161.53.68.185 '])
mex CAN.c bci.c ipci.c pc_i.c bapfw165.c filterlist.c bapfw320.c ipci_lin.c bapfw161.c dpram.c kiwi.c -largeArrayDims CFLAGS="\$CFLAGS -std=c99"

unix('scp root@161.53.68.185:q.mat ~/Documents/MATLAB/seminar/odzivi2');
unix('scp root@161.53.68.185:u.mat ~/Documents/MATLAB/seminar/odzivi2');
%
num=11;
unix(sprintf('scp root@161.53.68.185:q.mat ~/Documents/MATLAB/seminar/simulink/Data/q%d.mat',num));
unix(sprintf('scp root@161.53.68.185:u.mat ~/Documents/MATLAB/seminar/simulink/Data/u%d.mat',num));
unix(sprintf('scp root@161.53.68.185:ref.mat ~/Documents/MATLAB/seminar/simulink/Data/ref%d.mat',num));

%update note
save(sprintf('~/Documents/MATLAB/seminar/simulink/Data/%04d_note.mat',pokretanje),'note','pokretanje');
%3->[1,12]
%4->[5,40]
%5->[13,50]
%6->[8,50]
%7->[15,70]
%8->[20,70]
%9->[25,70]
%10->[30,80]
%%
i=u(1,:)>72.5&u(1,:)<74;j=cumsum(ones(1,length(u(1,:))));j=j(i);
for i=1:25;fprintf('%016d\n',upr(6,i));end

signal=u(2,u(1,:)>4&u(1,:)<9);plotaj_spektar;

%signal = zeljeni signal;
N = length(signal);
fs = 1/Ts; % 62.5 samples per second
fnyquist = fs/2; %Nyquist frequency

%% Single-sided magnitiude spectrum in decibels and Hertz
X_mags = abs(fft(signal));
bin_vals = [0 : N-1];
fax_Hz = bin_vals*fs/N;
N_2 = ceil(N/2);
plot(fax_Hz(1:N_2), 10*log10(X_mags(1:N_2)))
xlabel('Frequency (Hz)')
ylabel('Magnitude (dB)');
title('Single-sided Magnitude spectrum (Hertz)');
axis tight


%% identifikacija prijenosne funkcije struje

info=stepinfo(struja(2,struja(1,:)>=6&struja(1,:)<6.35),struja(1,struja(1,:)>=6&struja(1,:)<6.35),struja(2,struja(1,:)==6.34));
wn_i=1.8/info.RiseTime
zeta_i=sqrt(1-(pi/wn_i/(info.PeakTime-6))^2)
GI=tf(wn_i^2,[1,2*zeta_i*wn_i,wn_i^2])


