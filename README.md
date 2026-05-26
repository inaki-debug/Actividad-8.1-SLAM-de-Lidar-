# Actividad-8.1-SLAM-de-Lidar-

El objetivo de esta actividad es implementar un código en MATLAB el cual contiene la simuación de un robot con un LiDAR el cual debe seguir una trayectoria determinada por waypoints. Se debe encontrar los valores óptimos de las variables del robot para realizar la trayectoria de manera satisfactoria.

Para lograr el objetivo, se desarrolló el siguiente algoritmo:

1. Se definió la cinemática del robot con un radio de 0.1m de las ruedas y una distancia entre ellas de 0.5m. Este robot es de tipo diferencial.
2. Se estableció un tiempo de simulación `sampleTime` de 0.05s el cual genera un vector de tiempo para controlar la duración de los pasos de la simulación.
3. Se asigna la posición inicial del robot y se establecen dos mapas de la biblioteca que son exampleMap y complexMap los cuales tienen su determinado nivel de dificultad.
4. Para la configuración del LiDAR, se configuró de manera que escanea con un rango -180° a 180° o -pi a pi. La distancia máxima fue sintonizandose dependiendo del rendimiento del seguimiento de trayectoria.
5. Se configuró una matriz de coordenadas secuenciales que significan los waypoints. Estos se configuraron dependiendo de la trayectoria que se desea seguir.
6. Para el control de robot se configuró un Pure Pursuit el cual calcula constantemente las velocidades para llegar a los waypoints. Este tipo de controlador tiene los siguientes parámetros:
   - Lookahead Distance
   - Velocidad lineal deseada
   - Velocidad angular máxima
7. El algoritmo de evasión de obstaculos se integra el Histograma de Campo Vectorial para la navegación reactiva. Se configuran los límites de distancia del sensor, el número de sectores angulares, los umbrales del histograma, y los márgenes de seguridad.

Las variables que se ajustaron en cada ejercicio fueron: `sampleTime, tVec, initPose, lidar.scanAngles, lidar.maxRange, waypoints, controller.LookaheadDistance, controller.DesiredLinearVelocity y controller.MaxAngularVelocity`. 

### Ejercicio 1 (exampleMap)

Para este ejercicio se establecieron los siguientes waypoints:

```
waypoints = [initPose(4:3)'; 
             4 6;
             9 8;
             9 6;
             9 2;];
```
Y se sintonizaron las siguientes variables:

```
sampleTime = 0.05;              % Sample time [s]
tVec = 0:sampleTime:200;        % Time array

initPose = [4;3;0];            % Initial pose (x y theta)

lidar.maxRange = 1;%5

controller.LookaheadDistance = 0.5;%0.5
controller.DesiredLinearVelocity = 0.75; %0.75
controller.MaxAngularVelocity = 200
```

<img width="912" height="589" alt="imagen" src="https://github.com/user-attachments/assets/85e75689-093f-40a6-b861-acd9115f3e72" />

### Ejercicio 1 (complexMap)

Para este ejercicio se establecieron los siguientes waypoints:

```
waypoints = [initPose(4:2)'; 
             4, 7;
             9, 7;
              9, 2
             ];
```
Y se sintonizaron las siguientes variables:

```
sampleTime = 0.05;              % Sample time [s]
tVec = 0:sampleTime:200;        % Time array

initPose = [4;2;0];            % Initial pose (x y theta)

lidar.maxRange = 1;%5

controller.LookaheadDistance = 0.5;%0.5
controller.DesiredLinearVelocity = 0.75; %0.75
controller.MaxAngularVelocity = 200
```

<img width="912" height="589" alt="imagen" src="https://github.com/user-attachments/assets/87f948b4-cbf3-4469-b5c9-f55c00977d99" />

### Ejercicio 2 (exampleMap)

Para este ejercicio se establecieron los siguientes waypoints:

```
waypoints = [
            2,6;
            4,8;
            8,6;
            9,8;
            7,2;
            9,3];
```
Y se sintonizaron las siguientes variables:

```
sampleTime = 0.05;              % Sample time [s]
tVec = 0:sampleTime:200;        % Time array

initPose = [2;2;0];            % Initial pose (x y theta)

lidar.maxRange = 1;%5

controller.LookaheadDistance = 0.5;%0.5
controller.DesiredLinearVelocity = 0.75; %0.75
controller.MaxAngularVelocity = 200
```

<img width="912" height="589" alt="imagen" src="https://github.com/user-attachments/assets/655e2407-948e-4fb8-a969-46556f732651" />

### Ejercicio 2 (complexMap)

Para este ejercicio se establecieron los siguientes waypoints:

```
waypoints = [2, 2;
            5 4.5; 
            7, 6;
            4, 8; 
            9, 8; 
            9, 3;
            7, 2;
           ];
```
Y se sintonizaron las siguientes variables:

```
sampleTime = 0.05;              % Sample time [s]
tVec = 0:sampleTime:200;        % Time array

initPose = [2;2;0];            % Initial pose (x y theta)

lidar.maxRange = 0.5;%5

controller.LookaheadDistance = 0.5;%0.5
controller.DesiredLinearVelocity = 0.75; %0.75
controller.MaxAngularVelocity = 200
```

<img width="912" height="589" alt="imagen" src="https://github.com/user-attachments/assets/32930d15-2143-4de9-bd92-0a757e2e5b6b" />

### Ejercicio 3 (exampleMap)

Para este ejercicio se establecieron los siguientes waypoints:

```
waypoints = [
    1, 1;  
    1, 2;
    1, 3;
    1, 4;  
    2, 4;  
    2, 3;
    2, 2;
    2, 1;  
    3, 1;  
    3, 2;
    3, 3;
    3, 4;  
    4, 4;  
    4, 3;
    4, 2;
    4, 1];
      
```
Y se sintonizaron las siguientes variables:

```
sampleTime = 0.05;              % Sample time [s]
tVec = 0:sampleTime:200;        % Time array

initPose = [1;1;0];            % Initial pose (x y theta)

lidar.maxRange = 0.5;%5

controller.LookaheadDistance = 0.5;%0.5
controller.DesiredLinearVelocity = 0.75; %0.75
controller.MaxAngularVelocity = 200
```

<img width="912" height="589" alt="imagen" src="https://github.com/user-attachments/assets/d5b851f0-b630-487b-ad80-449ffd920680" />


### Ejercicio 3 (complexMap)

Para este ejercicio se establecieron los siguientes waypoints:

```
waypoints = [
    1, 2;  
    1, 1;
    2, 1;
    2, 2;
    3, 2;
    3, 1;
    4, 1;
    4, 2;
    4, 3;
    3, 3;
    4, 4;
    3, 4;
    2, 4;
    1, 4;
    1, 3;  
    2, 3];
      
```
Y se sintonizaron las siguientes variables:

```
sampleTime = 0.05;              % Sample time [s]
tVec = 0:sampleTime:200;        % Time array

initPose = [1;1;0];            % Initial pose (x y theta)

lidar.maxRange = 0.5;%5

controller.LookaheadDistance = 0.5;%0.5
controller.DesiredLinearVelocity = 0.75; %0.75
controller.MaxAngularVelocity = 200
```


<img width="912" height="589" alt="imagen" src="https://github.com/user-attachments/assets/8ccf1d07-b2cd-4b2c-a086-905c94e72b22" />


# Actividad 8.2 SLAM de LiDAR

En esta actividad se realizó una trayectoria con los siguientes waypoints: `(1, 2), (2, 10), (11, 8), (8, 2), y (1, 2)`. Como en el ejercicio anterior, se establecieron algunos waypoints de apoyo para realizar la trayectoria de manera satisfactoria. 

Para el exampleMap se utilizaron los siguientes waypoints:

```
waypoints = [
    1,2;
    4, 6;
    2,10;
    11,8;
    6,1;
    8,2;
    8.5,6;
    6, 8;
    1,2];
```

<img width="912" height="589" alt="imagen" src="https://github.com/user-attachments/assets/ebd778b6-cccb-4c7f-a074-14f890b8ed2a" />

Para el complexMap se utilizaron los siguientes waypoints:

```
waypoints = [
    1,2;
    4, 4;
    7, 6;
    2,10;
    7,6.5;
    10, 4;
    13, 6;
    11, 8;
    13, 6;
    10, 4;
    8, 2;
    7.5, 4;
    4, 4;
    1, 2;]
```

<img width="912" height="589" alt="imagen" src="https://github.com/user-attachments/assets/ef02a91f-edfa-41de-bf78-9fc946356111" />









