
% sim_planta_pid_octave.m
%---------------------------------------------
% Planta NEMA-17 (½-step) + filtro EMA (α = 0.2)
% PI-D diseñado para:
%   • t_p ≥ 10 s  
%   • M_p < 2 %  
%   • t_s ≥ 10 s  
%---------------------------------------------
pkg load control   % sólo en Octave

%% Parámetros de la planta y simulación
Ts  = 0.05;             % periodo de muestreo [s]
Keq = 0.045;            % 0.9°/µ-step · Ts
Gz  = tf(Keq, [1 -1], Ts);       % integrador discreto
Hz  = tf(0.2, [1 -0.8], Ts);     % filtro EMA
Pz  = series(Hz, Gz);            % planta

%% Diseño del controlador
Kp    = 0.35;   Ki = 0.005;   Kd = 0.1;   alpha = 0.5;

% Termino P+I discreto
Cpi = Kp + Ki*Ts*tf([1 0], [1 -1], Ts);

% Termino D discreto con filtro 1er orden:
%   Dz(z) = (Kd*(1-α)/Ts) · (1 - z^-1)/(1 - α z^-1)
bD = Kd*(1 - alpha)/Ts * [1 -1];
aD = [1 -alpha];
Dz = tf(bD, aD, Ts);

% Controlador completo
Cz = Cpi + Dz;

%% Lazo cerrado
CL = feedback(Cz * Pz, 1);

%% Simulación escalón
Tf = 40;                       % tiempo total de simulación [s]
t  = 0:Ts:Tf;
r  = ones(size(t));           % escalón unitario de 1°

y = lsim(CL, r, t);

%% Simular señal de control u[k]

% Entrada: escalón unitario
r = ones(size(t));        % referencia
e = r - y;                % error

% Inicializar vectores
u  = zeros(size(t));
ei = 0;                   % integral acumulada
ed_ant = 0;               % derivada anterior

for k = 2:length(t)
  % Error
  e_k = e(k);
  
  % Integral (suma acumulada)
  ei += e_k * Ts;

  % Derivada (diferencia con filtro)
  ed  = (e(k) - e(k-1));
  edf = (1 - alpha)*ed + alpha*ed_ant;
  ed_ant = edf;

  % Control PID (discreto)
  u(k) = Kp * e_k + Ki * ei + Kd * edf;
end

%% Graficar señal de control
figure(2); clf; hold on; grid on;
stairs(t, u, 'r', 'LineWidth', 1.3);
xlabel('Tiempo [s]');
ylabel('u[k]');
title('Señal de control (PID discreto)');

[u_max, idx_max] = max(u)
t_max = t(idx_max)
fprintf("u_max = %.4f en t = %.2f s\n", u_max, t_max)



%% Cálculo manual de métricas
y_inf = y(end);                     % valor final (≈ ganancia DC)
% --- tiempo al pico y sobreimpulso ---
[ypk, idx_pk] = max(y);
tp = t(idx_pk);
Mp = (ypk - y_inf)/y_inf * 100;     % %

% --- tiempo de asentamiento (±2 %) ---
band = 0.02 * y_inf;
ts  = NaN;
for k = 1:length(t)
  if all( abs(y(k:end) - y_inf) <= band )
    ts = t(k);
    break;
  end
end

% --- rise‑time opcional (10‑90 %) ---
y10 = 0.10*y_inf;  y90 = 0.90*y_inf;
idx10 = find(y >= y10, 1);   idx90 = find(y >= y90, 1);
tr = t(idx90) - t(idx10);

%% Mostramos resultados
printf("Resultados:\n");
printf("  t_p  = %.2f s\n", tp);
printf("  M_p  = %.2f %%\n", Mp);
if !isnan(ts)
  printf("  t_s  = %.2f s (±2%%)\n", ts);
else
  printf("  t_s  > %.2f s  (no estabiliza en el intervalo)\n", Tf);
endif
printf("  t_r  = %.2f s (10‑90%%)\n", tr);
%% Gráfica de la respuesta
figure(1); clf; hold on; grid on
stairs(t, y,   'b',  'LineWidth', 1.4);
plot(   t, r,  'k--','LineWidth', 1);
y_inf = dcgain(CL);
band = 0.02*y_inf;
plot([0 Tf],[y_inf y_inf],    'r--');
plot([0 Tf],[y_inf+band y_inf+band], 'r:');
plot([0 Tf],[y_inf-band y_inf-band], 'r:');
plot([tp tp],[min(y) max(y)],  'm--');

text(tp+0.5, y_inf+0.05, sprintf("t_p = %.1f s", tp), 'Color','m');
text(ts+0.5, y_inf+0.05, sprintf("t_s = %.1f s", ts), 'Color','r');

xlabel('Tiempo [s]');
ylabel('Salida \theta [°]');
title('Respuesta al escalón del lazo cerrado');
legend({'y(t)','r(t)'}, 'Location','SouthEast');
