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


