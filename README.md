# Odometria de karts con el uso de dispositivos moviles
En el presente repositorio se describe el proceso de tratamiento de datos obtenidos de un recorrido en karts a la plazoleta del edificio CyT en la Universidad Nacional de Colombia, mediante el uso de sensores integrados en dispositivos móviles, con el fin de reconstruir la trayectoria seguida durante el recorrido.
## Obtención de datos
```matlab
jsonStr = fileread('Datos_Carlos.json');
DatosJson = jsondecode(jsonStr);
Valores = DatosJson.payload.values;
```

```matlab
Fs = 62.5;
Ts = 1/Fs;
accO = [Valores(72:end,1) Valores(72:end,2) Valores(72:end,3)];
gyrsF = [Valores(72:end,4) Valores(72:end,5) Valores(72:end,6)];
L = size(accO); 
t = (0:L(1)-1)*Ts;
```

## Diseño del filtro
Una vez tenemos los datos del recorrido, diseñamos un filtro para reducir el ruido de las aceleraciones y del giróscopo, para ello primero graficamos el espectro de frecuencias e identificamos la frecuencia en la que se concentra la mayor parte de la potencia de la señal.

```matlab
FrecAccsF = abs(fft(accO));
FrecGyrsF = abs(fft(gyrsF));
frec = Fs/L(1)*(0:L(1)-1);
```

<p align="center">
   <img src="/Imágenes/Esp_f.png" alt="Espectro de Frecuencias" width="500"><br> 

Como se puede ver en la gráfica, la mayor parte de la potencia de la señal se encuentra por debajo de los 10 Hz, mientras que las componentes de frecuencias superiores contribuyen al ruido en la señal. Por lo cual se debe implementar un filtro pasabajos con frecuencia de corte en 10 Hz para reducir el ruido presente en la señal.

Los parámetros necesarios para caracterizar dicho filtro son: 

+ Banda de Transición ($\Delta \omega$): Representa el intervalo de frecuencias permisibles para que el filtro tenga el efecto deseado en la señal. Es importante aclarar que por el teorema de muestreo de Nyquist cada período se toman dos muestras, por lo cual las frecuencias se deben normalizar usando la siguiente relación:

$$f_N = \frac{f}{2f_s}$$

+ Tolerancia de paso ($\Delta_1$): Representa la magnitud mínima que el filtro le permite reducir a la señal dentro de los límites de la banda de paso. También suele representarse en decibeles mediante la siguiente relación:

$$R_p = 20log_{10}(\frac{1}{1-\Delta_1})$$

+ Tolerancia de rechazo ($\Delta_2$): Representa la magnitud de corte máxima que el filtro debe aplicar a la señal, una vez se sobrepase el límite superior de la banda de transición. Al igual que en la tolerancia de paso, suele representarse en decibeles con la siguiente relación:

$$A_s = 20log_{10}(\frac{1}{\Delta_2})$$

+ Orden del filtro (N): Es un número entero que determina la cantidad de muestras y retardos que emplea el filtro para procesar la señal, entre mayor sea el orden se satisfacen parámetros más estrictos pero se eleva considerablemente el costo computacional, se puede lograr una aproximación del orden que cumpla medianamente bien con los requerimientos en los demás parámetros mediante la siguiente ecuación:

$$N \approx \frac{f_sA_s}{22\Delta f}$$

Teniendo en cuenta los parámetros anteriores, se propone un filtro FIR pasabajos en Matlab con frecuencia normalizada de corte en Hz y ventana de Chebysev de orden n y tolerancia de rechazo $A_s$

```matlab
DeltaT = 1;
DeltaR = 10^(-1);
As = 20*log10(1/DeltaR);
n = round(Fs*As/(DeltaT*22));
f = 10/(2*Fs);
filtro = fir1(n,f,"low",chebwin(n+1,As));
acc = filter(filtro,1,accO);
gyr = filter(filtro,1,gyrsF);
```

## Eliminación del efecto de la gravedad

Como se empleó una IMU para adquirir los datos del recorrido en karts, esta también considera las aceleraciones de la gravedad sobre el dispositivo movil, por lo cual se debe eliminar dicho efecto sobre el vector total de aceleracione, para ello se asume que el vector de aceleración después del filtro es una combinación líneal de la aceleración deseada con un vector totalmente vertical de magnitud igual a 9.8 $m/s^2$ que representa la gravedad, en este sentido la relación entre ambos vectores puede ser descrita por el coseno de ambas magnitudes.

```matlab
accTot = sqrt(acc(:,1).^2+acc(:,2).^2+acc(:,3).^2);
cosGrav = 9.81/accTot;
cosD = [acc(:,1)./accTot acc(:,2)./accTot acc(:,3)./accTot];
accsG = sqrt(abs(accTot.^2-9.81^2)).*[cosD(:,1) cosD(:,2) cosD(:,3)];
```

## Obtención de la velocidad y posición angular

```matlab
for j=1:L(2)
    Ang(1,j) = 0;
    for i=1:L(1)-1
        Ang(i+1,j) = Ang(i,j)+Ts*gyr(i,j);
    end
end
```

```matlab
for j=1:L(2)
    Vel(1,j) = 0;
    for i=1:L(1)-1
        Vel(i+1,j) = Vel(i,j)+Ts*accsG(i,j);
    end
end
```

## Obtención de la trayectoria

```matlab
for j=1:L(2)
    Pos(1,j) = 0;
    for i=1:L(1)-1
        Pos(i+1,j) = Pos(i,j)+Ts*Vel(i,j);
    end
end
```
