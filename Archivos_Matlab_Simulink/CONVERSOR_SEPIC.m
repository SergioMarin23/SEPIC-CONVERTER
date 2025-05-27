clc;
clear;
close all;

%Variables
Vin_max = 17.4;
Vout = 12;
Iin_max = 5.02;
Fs = 100000;
Ts = 1/(Fs);
%Ts = 80e-6;
%Fs = 1/Ts;

%Calculos iniciales
Duty = Vout/(Vin_max + Vout);

if Duty >= 0.20 && Duty < 0.30
    n_efi = 0.80;
elseif Duty >= 0.30 && Duty < 0.50
    n_efi = 0.90;
elseif Duty >= 0.50 && Duty <= 0.60
    n_efi = 0.80;
end

Powr_in = Vin_max*Iin_max;
Powr_out = n_efi*Powr_in;
Iout_max = (n_efi*Vin_max*Iin_max)/(Vout);
R_load = (Vout.^2)/Powr_out;

%{
Powr = Vout*Iout_max;
Iin_max = Powr/Vin_max;
%}

%Rizados
Del_IL1 = 0.15*Iin_max;
Del_IL2 = 0.15*Iout_max;
Del_Vc1 = 0.15*Vin_max;
Del_Vc2 = 0.15*Vout;

%Calculos componentes
L1 = (Vout*Duty)/(Del_IL1*Fs);
L2 = (Vout*Duty)/(Del_IL2*Fs);
Cap1 = (Vout*Duty)/(R_load*Del_Vc1*Fs);
Cap2 = 10*((Vout*Duty)/(R_load*Del_Vc2*Fs));

%%Matrices para estado ON y OFF
A1 = [0 0 0 0; 
      0 0 -1/(L2) 0; 
      0 1/Cap1 0 0; 
      0 0 0 -1/(R_load*Cap2)];
A2 = [0 0 -1/(L1) -1/(L1); 
      0 0 0 1/(L2); 
      1/Cap1 0 0 0; 
      1/Cap2 -1/Cap2 0 -1/(R_load*Cap2)];
B1 = [1/L1; 
        0;
        0; 
        0];
B2 = B1;
C1 = [0 0 0 1];
C2 = C1;
D1 = 0;
D2 = D1;

%Espacio de estados promedio
A = Duty.*A1 + (1-Duty).*A2;
B = Duty.*B1 + (1-Duty).*B2;
C = Duty.*C1 + (1-Duty).*C2;
D = Duty.*D1 + (1-Duty).*D2;

%Nuevas matrices
%{
A = [0 0 -(1-D)/L1 -(1-D)/L1;
     0 0 D/L2 -(1-D)/L2;
     (1-D)/Cap1 -D/Cap1 0 0;
     (1-D)/Cap2 (1-D)/Cap2 0 -1/(R*Cap2)];
B = [1/L1;
     0;
     0;
     0];
C = [0 0 0 1];
E = 0;
%}

%
%Primera forma función de transferencia
%Función de transferencia continua y discreta
[num,den] = ss2tf(A,B,C,D);

Gp_cont = tf(num,den);
Gp_disc = c2d(Gp_cont, Ts,'zoh');
[num_d,den_d] = tfdata(Gp_disc,'verbose');

%Respuesta al escalon
figure(1)
step(Gp_cont,'r-', Gp_disc,'b-')
grid on;
%xlim([0 7e-3])
%ylim([0 3])
grid on;
%}


%Calculo LQR
%{
R = rand(1);
Q = diag(rand(4,1));
K = lqr(A,B,Q,R);
sys = ss(A-B*K,B,C,D);

figure(2)
step(sys)
grid on;

%Valores que pueden funcionar
%{
K1 = [7.5565    9.3810    0.5375    0.0589];
K2 = [7.9538   11.3307    0.6301    0.0542];
K3 = [8.0711    9.9112    0.5895    0.0631];
K4 = [7.9038   10.4785    0.5991    0.0585];
K5 = [7.2059    7.5640    0.4634    0.0655];
%}
%}

