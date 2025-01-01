# Odometria de karts con el uso de dispositivos moviles
En el presente repositorio se describe el proceso de tratamiento de datos obtenidos de un recorrido en karts mediante el uso de sensores integrados en dispositivos móviles, con el fin de reconstruir la trayectoria seguida durante el recorrido.
# Diseño de Filtros
Una vez tenemos los datos del recorrido, diseñamos un filtro para reducir el ruido de las aceleraciones, para ello primero graficamos el espectro de frecuencias e identificamos la frecuencia en la que se ecuentra la mayor parte de la señal.

```matlab
FrecAccsF = abs(fft(accO));
FrecGyrsF = abs(fft(gyrsF));
frec = Fs/L(1)*(0:L(1)-1);
```
