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

% Transformada de Fourier
FrecAccsF = abs(fft(accO));
FrecGyrsF = abs(fft(gyrsF));
frec = Fs/L(1)*(0:L(1)-1);

% Graficar Espectros de Frecuencias
figure
plot(frec,FrecAccsF(:,1))
title("Espectro de frecuencias aceleración");
xlim([0 63])
xlabel("f (Hz)");
ylabel("|A(j\omega)|");

figure
plot(frec,FrecGyrsF(:,1))
title("Espectro de frecuencias velocidad angular");
xlim([0 63])
xlabel("f (Hz)");
ylabel("|\Omega(j\omega)|");

%Filtro pasa bajo FIR
DeltaT = 1;
DeltaR = 10^(-1);
As = 20*log10(1/DeltaR);
n = round(Fs*As/(DeltaT*22));
f = 10/(2*Fs);
filtro = fir1(n,f,"low",chebwin(n+1,As));
acc = filter(filtro,1,accO);
gyr = filter(filtro,1,gyrsF);

%Graficar señal original vs filtrada
figure
subplot(2,1,1)
plot(t,accO(:,1))
title('Señal Original')
ylabel('Aceleracion (m/s^2)')
ys = ylim;

subplot(2,1,2)
plot(t,acc(:,1))
title('Señal Filtrada')
ylabel('Aceleracion (m/s^2)')
xlabel('Tiempo (s)')
ylim(ys)

figure
subplot(2,1,1)
plot(t,gyrsF(:,1))
title('Señal Original')
ylabel('Velocidad Angular (rad/s)')
ys = ylim;

subplot(2,1,2)
plot(t,gyr(:,1))
title('Señal Filtrada')
ylabel('Velocidad Angular (rad/s)')
xlabel('Tiempo (s)')
ylim(ys)

%Eliminacion del efecto de la gravedad
accTot = sqrt(acc(:,1).^2+acc(:,2).^2+acc(:,3).^2);
cosGrav = 9.81./accTot;
MagAccD  = sqrt(abs(accTot.^2+9.81^2-2.*cosGrav.*accTot.*9.81));
senAlpha = 9.81.*sin(abs(acos(cosGrav)))./accTot;
cosD = [acc(:,1)./accTot acc(:,2)./accTot acc(:,3)./accTot];
accsG = MagAccD.*senAlpha.*cosD;

%Obtencion de la matriz Angular
for j=1:L(2)
    Ang(1,j) = 0;
    for i=1:L(1)-1
        Ang(i+1,j) = Ang(i,j)+Ts*gyr(i,j);
    end
end

%Obtencion de la matriz velocidad
for j=1:L(2)
    Vel(1,j) = 0;
    for i=1:L(1)-1
        Vel(i+1,j) = Vel(i,j)+Ts*accsG(i,j);
    end
end

%Graficar posiciones angulares
figure
plot(t,Vel(:,1))
title('Velocidad en x')
xlabel('Tiempo (s)')
ylabel('v_x (m/s)')

figure
plot(t,Vel(:,2))
title('Velocidad en y')
xlabel('Tiempo (s)')
ylabel('v_y (m/s)')

%Graficar velocidades
figure
plot(t,Vel(:,1))
title('Velocidad en x')
xlabel('Tiempo (s)')
ylabel('v_x (m/s)')

figure
plot(t,Vel(:,2))
title('Velocidad en y')
xlabel('Tiempo (s)')
ylabel('v_y (m/s)')

%Obtencion de la matriz posición
for j=1:L(2)
    Pos(1,j) = 0;
    for i=1:L(1)-1
        Pos(i+1,j) = Pos(i,j)+Ts*Vel(i,j);
    end
end

%Graficar trayectoria
figure
plot(Pos(:,1),Pos(:,2))
title('Trayectoria')
xlabel("r_x (m)");
ylabel("r_z (m)");

