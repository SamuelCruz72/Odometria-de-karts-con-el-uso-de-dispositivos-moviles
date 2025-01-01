# Odometria de karts con el uso de dispositivos moviles
En el presente repositorio se describe el proceso de tratamiento de datos obtenidos de un recorrido en karts mediante el uso de sensores integrados en dispositivos móviles, con el fin de reconstruir la trayectoria seguida durante el recorrido.
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
Una vez tenemos los datos del recorrido, diseñamos un filtro para reducir el ruido de las aceleraciones, para ello primero graficamos el espectro de frecuencias e identificamos la frecuencia en la que se ecuentra la mayor parte de la señal.

```matlab
FrecAccsF = abs(fft(accO));
FrecGyrsF = abs(fft(gyrsF));
frec = Fs/L(1)*(0:L(1)-1);
```

<p align="center">
   <img src="/Imágenes/Esp_f.png" alt="Espectro de Frecuencias" width="500"><br> 

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
