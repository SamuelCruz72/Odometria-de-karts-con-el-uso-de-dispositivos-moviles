# Odometría de karts con el uso de dispositivos móviles
En el presente repositorio se describe el proceso de tratamiento de datos obtenidos de un recorrido en karts a la plazoleta del edificio CyT en la Universidad Nacional de Colombia, mediante el uso de sensores integrados en dispositivos móviles, con el fin de reconstruir la trayectoria seguida durante el recorrido.

## Obtención de datos
Para obtener los datos se usa una plataforma denominada Edge Impulse a la cual se conecta el dispositivo móvil con el cual se va tomar la medición; una vez conectado, se fija el tiempo de muestreo y se selecciona el tiempo de medición tal y como se indica en la siguiente imagen:

<p align="center">
   <img src="/Imágenes/Edge Impulse.png" alt="Edge impulse" width="500"><br> 

Una vez se tienen los datos, se descargan en formato JSON y se cargan al programa Matlab para analizarlos; para poder leer los datos en formato numérico se emplea el siguiente código, donde el parámetro de la función ```fileread()``` es el nombre del archivo que contenga los datos JSON:

```matlab
jsonStr = fileread('Datos_Carlos.json');
DatosJson = jsondecode(jsonStr);
Valores = DatosJson.payload.values;
```

El paso siguiente es definir parámetros generales de los datos obtendos tales como la frecuencia de muestreo y el vector de tiempo, a su vez se definen los vectores de aceleración y velocidad angular obtenidos a partir de la llave de valores legibles del archivo JSON:

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
   <img src="/Imágenes/Esp_f.png" alt="Espectro de Frecuencias Aceleracion" width="500"><br> 

<p align="center">
   <img src="/Imágenes/Esp_g.png" alt="Espectro de Frecuencias Velocidad Angular" width="500"><br> 

Como se puede ver en las gráficas, la mayor parte de la potencia de la señal se encuentra por debajo de los 10 Hz, mientras que las componentes de frecuencias superiores contribuyen al ruido en la señal. Por lo cual se debe implementar un filtro pasabajos con frecuencia de corte en 10 Hz para reducir el ruido presente en la señal.

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

Con lo cual, los resultados de la aplicación del filtro en los vectores de aceleración y velocidad angular son:

<p align="center">
   <img src="/Imágenes/Facc.png" alt="Aceleracion Filtrada" width="500"><br> 

<p align="center">
   <img src="/Imágenes/Fgyr.png" alt="Velocidad Angular Filtrada" width="500"><br> 

## Eliminación del efecto de la gravedad
Como se empleó una IMU para adquirir los datos del recorrido en karts, esta también considera las aceleraciones de la gravedad sobre el dispositivo movil, por lo cual se debe eliminar dicho efecto sobre el vector total de aceleraciones, para ello se asume que el vector de aceleración después del filtro es una combinación líneal de la aceleración deseada con un vector totalmente vertical de magnitud igual a 9.8 $m/s^2$ que representa la gravedad, en este sentido la relación entre ambos vectores puede ser descrita por el coseno de ambas magnitudes.

Ahora bien, dada la suposición de que la aceleración medida por la IMU es una combinación lineal del vector de aceleración desdeado y el vector de gravedad, se puede establecer una relación entre las magnitudes de los tres vectores, empleando la ley del coseno:

$$a_d =\sqrt{a_m^2+g^2-2a_mgcos(\gamma)}$$

Una vez conocida la magnitud de la aceleración deseada, usando la ley de senos se puede conocer su dirección relativa a la aceleración medida y con el uso de los cosenos directres de esta se determina su dirección absoluta:

$$sen(\alpha) = \frac{gsen(\gamma)}{a_d}$$

```matlab
accTot = sqrt(acc(:,1).^2+acc(:,2).^2+acc(:,3).^2);
cosGrav = 9.81/accTot;
MagaccsgG  = sqrt(abs(accTot.^2+9.81^2-2.*cosGrav.*accTot.*9.81));
cosD = [acc(:,1)./accTot acc(:,2)./accTot acc(:,3)./accTot];
accsG = MagaccsgG*cosD;
```

## Obtención de la velocidad y posición angular
Una vez tenemos filtrados los datos del vector de velocidad angular dado por las mediciones del giróscopo, se obtiene el vector de posición angular teniendo en cuenta que ambas magnitudes se relacionan mediante la siguiente ecuación diferencial:

$$\frac{d\vec{\theta}(t)}{dt}=\vec{\omega}(t)$$

Para resolver esta ecuación empleamos el método de Euler teniendo en cuenta que el paso h viene dado por el tiempo de muestreo de nuestro dispositivo móvil y la posición angular inicial del kart es 0 rad/s:

$$\vec{\theta}_{k+1}=\vec{\theta}_k+T_s\vec{\omega}_k$$

```matlab
for j=1:L(2)
    Ang(1,j) = 0;
    for i=1:L(1)-1
        Ang(i+1,j) = Ang(i,j)+Ts*gyr(i,j);
    end
end
```

Al igual que en el anterior caso, la velocidad y la aceleración del kart en coordenadas cartesianas se relacionan mediante una ecuación diferencial dependiente del tiempo:

$$\frac{d\vec{v}(t)}{dt}=\vec{a}(t)$$

Para resolver esta ecuación también empleamos el método de Euler teniendo en cuenta las consideraciones del paso y condiciones iniciales iguales a cero:

$$\vec{v}_{k+1}=\vec{v}_k+T_s\vec{a}_k$$

```matlab
for j=1:L(2)
    Vel(1,j) = 0;
    for i=1:L(1)-1
        Vel(i+1,j) = Vel(i,j)+Ts*accsG(i,j);
    end
end
```

## Obtención de la trayectoria
Finalmente, para obtener la trayectoría del kart en coordenadas cartesianas se emplea el vector de velocidades obtenido en el paso anterior, teniendo en cuenta que su ecuación diferencial es:

$$\frac{d\vec{r}(t)}{dt}=\vec{v}(t)$$

Entonces su solución por método de Euler con paso definido y condiciones iniciales iguales a cero estaría dada por:

$$\vec{r}_{k+1}=\vec{r}_k+T_s\vec{v}_k$$

```matlab
for j=1:L(2)
    Pos(1,j) = 0;
    for i=1:L(1)-1
        Pos(i+1,j) = Pos(i,j)+Ts*Vel(i,j);
    end
end
```
