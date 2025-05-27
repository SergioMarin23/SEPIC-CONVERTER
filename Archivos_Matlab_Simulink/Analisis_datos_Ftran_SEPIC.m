clc; clear; close all

%LAZO ABIERTO
%
% lee la matriz de datos
A= xlsread('Datos_planta_SEPIC','Hoja1');
A = A';
time = A(1,:);
amplitud = A(2,:);
ultA= length(A);

%plot (time,amplitud);
%grid on;

%}

%% Modelo original (respuesta medida)
K = max(amplitud);  % Ganancia o estado estacionario
t = time;

%% MÉTODO 1 – 63.2% + retardo
%
L = 0.0120;  % Retardo conocido o estimado

A_tao = K * 0.632;
tao1 = NaN;
for i = 1:length(amplitud)-1
    if A_tao >= amplitud(i) && A_tao <= amplitud(i+1)
        tao1 = time(i);
        break;
    end
end
if isnan(tao1)
    error('Método 1: No se encontró el tiempo correspondiente al 63.2%% de la amplitud');
end

Gp_base = tf(K, [tao1 1]);
[num_pade, den_pade] = pade(L, 1);
Gp_delay = tf(num_pade, den_pade);
Gp_1 = series(Gp_delay, Gp_base);

[Y1, T1] = step(Gp_1, t);
Error_1 = abs(max(amplitud) - max(Y1)) * 100 / max(amplitud);
%}

%% MÉTODO 2 – Doble constante de tiempo por derivadas
%
A_tao1 = K * 0.284;
A_tao2 = K * 0.632;
[~, idx_inflexion] = max(diff(diff(amplitud)));
t_inflexion = time(idx_inflexion + 1);

taoA = NaN; taoB = NaN;
for h = 1:length(amplitud)-1
    if A_tao1 >= amplitud(h) && A_tao1 <= amplitud(h+1)
        taoA = time(h);
    end
    if A_tao2 >= amplitud(h) && A_tao2 <= amplitud(h+1)
        taoB = time(h);
    end
end
if isnan(taoA) || isnan(taoB)
    error('Método 2: No se encontraron los tiempos para 28.4%% y 63.2%%');
end

den2 = [taoA * taoB, taoA + taoB, 1];
Gp_2 = tf(K, den2, 'InputDelay', L);

[Y2, T2] = step(Gp_2, t);
Error_2 = abs(max(amplitud) - max(Y2)) * 100 / max(amplitud);
%}

%% MÉTODO 3 – Método de Smith
%
dA = diff(amplitud) ./ diff(t);
[~, idx_inflexion] = max(dA);
m = dA(idx_inflexion);
b = amplitud(idx_inflexion) - m * t(idx_inflexion);

X = [1/m, -b/m];
Tu = polyval(X, 0.1);
Ta = polyval(X, K);

Tabla = [1,1,0,0;
         2,2.718,0.282,0.104;
         3,3.095,0.805,0.218;
         4,4.463,1.425,0.319;
         5,5.119,2.100,0.410;
         6,5.669,2.811,0.493;
         7,6.226,3.547,0.570;
         8,6.711,4.307,0.642;
         9,7.164,5.031,0.709;
         10,7.590,5.869,0.773];
n = 4;  % Orden del sistema (puedes ajustarlo si lo deseas)

Tu_teorico = Tabla(n,4) * Ta;
T = Ta / Tabla(n,2);
T1_est = L + Tu - Tu_teorico;
T1_est = max(T1_est, 0);

den3 = [T 1];
for i = 1:n-1
    den3 = conv([T 1], den3);
end
Gp_3 = tf(K, den3, 'InputDelay', T1_est);

[Y3, T3] = step(Gp_3, t);
Error_3 = abs(max(amplitud) - max(Y3)) * 100 / max(amplitud);

%% Gráfica comparativa
figure;
plot(t, amplitud, 'k', 'LineWidth', 1.5, 'DisplayName', 'Datos reales');
hold on;
plot(T1, Y1, 'b--', 'LineWidth', 1.2, 'DisplayName', 'Modelo Método 1');
plot(T2, Y2, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Modelo Método 2');
plot(T3, Y3, 'g--', 'LineWidth', 1.2, 'DisplayName', 'Modelo Método 3');
xlabel('Tiempo (s)');
ylabel('Amplitud');
title('Comparación de modelos estimados');
legend('Location', 'best');
grid on;
%}
%% Resultados de error
%
disp(['Error Método 1: ', num2str(Error_1), ' %']);
disp(['Error Método 2: ', num2str(Error_2), ' %']);
disp(['Error Método 3: ', num2str(Error_3), ' %']);
%}

%% Discretización
%{
Fs = 100000;
Ts = 1/Fs;

Gp_1_dis = c2d(Gp_1, Ts,'zoh');
Gp_2_dis = c2d(Gp_2, Ts,'zoh');
Gp_3_dis = c2d(Gp_3, Ts,'zoh');
%}
