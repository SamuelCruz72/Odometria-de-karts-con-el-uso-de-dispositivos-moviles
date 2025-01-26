clc, clear
% Leer datos JSON
jsonStr = fileread('Datos_Carlos.json');
DatosJson = jsondecode(jsonStr);
Valores = DatosJson.payload.values;

% Definir vector de tiempo y matriz de aceleraciones
Fs = 62.5;
Ts = 1/Fs;
accO = [Valores(72:end,1) Valores(72:end,2) Valores(72:end,3)];
gyrsF = [Valores(72:end,4) Valores(72:end,5) Valores(72:end,6)];
L = size(accO); 
t = (0:L(1)-1)*Ts;
figure
plot(t,accO)
title("Aceleraciones medidas en el tiempo");
grid on
xlabel("Tiempo (s)");
ylabel("Aceleración (m/s^2)");
legend('a_x', 'a_y', 'a_z', 'Location', 'northeast')

figure
plot(t,accO)
title("Velocidades angulares medidas en el tiempo");
grid on
xlabel("Tiempo (s)");
ylabel("Velocidad Angular (rad/s)");
legend('\omega_x', '\omega_y', '\omega_z', 'Location', 'northeast')

% Transformada de Fourier
FrecAccsF = abs(fft(accO));
FrecGyrsF = abs(fft(gyrsF));
frec = Fs/L(1)*(0:L(1)-1);

% Graficar Espectros de Frecuencias
figure
plot(frec,FrecAccsF)
title("Espectro de frecuencias aceleración");
grid on
xlim([0 63])
xlabel("f (Hz)");
ylabel("|A(j\omega)|");
legend('A_x', 'A_y', 'A_z', 'Location', 'northeast')

figure
plot(frec,FrecGyrsF)
title("Espectro de frecuencias velocidad angular");
ax = gca;
ax.FontSize = 9;
grid on
xlim([0 63])
xlabel("f (Hz)");
ylabel("|\Omega(j\omega)|");
legend('\Omega_x', '\Omega_y', '\Omega_z', 'Location', 'northeast')

%Filtro pasa bajo FIR
DeltaT = 1;
DeltaR = 10^(-1);
As = 20*log10(1/DeltaR);
n = round(Fs*As/(DeltaT*22));
f = 10/(2*Fs);
filtro = fir1(n,f,"low",chebwin(n+1,As));
figure
freqz(filtro,1)
acc = filter(filtro,1,accO);
gyr = filter(filtro,1,gyrsF);

%Corrección desfase filtro FIR
delay = ceil(mean(grpdelay(filtro)));
tt = t(1:end-delay);
accn = accO(1:end-delay,:);
accf = acc;
accf(1:delay,:) = [];
gyrn = gyrsF(1:end-delay,:);
gyrf = gyr;
gyrf(1:delay,:) = [];
l = size(accf);

%Graficar señal original vs filtrada
figure
subplot(2,1,1)
plot(tt,accn)
grid on
ylim([-10 30])
title('Señales Originales')
ax = gca;
ax.FontSize = 8;
ylabel('Aceleración (m/s^2)')
legend('a_x', 'a_y', 'a_z', 'Location', 'northeast')
ys = ylim;

subplot(2,1,2)
plot(tt,accf)
grid on
title('Señales Filtradas')
ax = gca;
ax.FontSize = 8;
ylabel('Aceleración (m/s^2)')
xlabel('Tiempo (s)')
ylim(ys)
legend('a_x', 'a_y', 'a_z', 'Location', 'northeast')

figure
subplot(2,1,1)
plot(tt,gyrn)
grid on
title('Señales Originales')
ax = gca;
ax.FontSize = 8;
ylabel('Velocidad Angular (rad/s)')
legend('\omega_x', '\omega_y', '\omega_z', 'Location', 'northeast')
ys = ylim;

subplot(2,1,2)
plot(tt,gyrf)
grid on
title('Señales Filtradas')
ax = gca;
ax.FontSize = 8;
ylabel('Velocidad Angular (rad/s)')
xlabel('Tiempo (s)')
ylim(ys)
legend('\omega_x', '\omega_y', '\omega_z', 'Location', 'northeast')

%Eliminacion del efecto de la gravedad
accTot = sqrt(accf(:,1).^2+accf(:,2).^2+accf(:,3).^2);
Gamma = acos(9.81./accTot);
MagAccD  = sqrt(abs(accTot.^2-9.81^2));
cosD = [accf(:,1)./accTot accf(:,2)./accTot accf(:,3)./accTot];
accsG = MagAccD.*sin(Gamma).*cosD;

figure
plot(tt,accsG)
grid on
title('Aceleración Tangencial vs Tiempo')
xlabel('Tiempo (s)')
ylabel('a (m/s^2)')
legend('a_x', 'a_y', 'a_z', 'Location', 'southeast')

%Obtencion de la matriz Angular
for j=1:l(2)
    Ang(1,j) = 0;
    for i=1:l(1)-1
        Ang(i+1,j) = Ang(i,j)+Ts*gyr(i,j);
    end
end

%Obtencion de la matriz velocidad
for j=1:l(2)
    Vel(1,j) = 0;
    for i=1:l(1)-1
        Vel(i+1,j) = Vel(i,j)+Ts*accsG(i,j);
    end
end

%Graficar posiciones angulares
figure
plot(tt,Ang)
grid on
title('Angulo vs Tiempo')
xlabel('Tiempo (s)')
ylabel('\theta (rad)')
legend('\theta_x', '\theta_y', '\theta_z', 'Location', 'southeast')

%Graficar velocidades
figure
plot(tt,Vel)
grid on
title('Velocidad vs Tiempo')
xlabel('Tiempo (s)')
ylabel('Velocidad (m/s)')
legend('v_x', 'v_y', 'v_z', 'Location', 'southeast')

%Obtencion de la matriz posición
for j=1:l(2)
    Pos(1,j) = 0;
end
for i=1:l(1)-1
    Pos(i+1,:) = Pos(i,:)+Ts.*(Vel(i,:)*[1 0 0; 0 cos(Ang(i,1)) sin(Ang(i,1)); 0 -sin(Ang(i,1)) cos(Ang(i,1))]);
end

%Graficar trayectoria
figure
plot(Pos(:,3),Pos(:,2))
grid on
title('Trayectoria')
xlabel("r_z (m)");
ylabel("r_y (m)");

