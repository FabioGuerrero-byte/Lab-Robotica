# celda_aceite — gemelo digital de la línea de envasado (Lab 1 v2)

Un solo Gazebo (Harmonic / ROS 2 Jazzy) con los 5 robots del plano, las estaciones y una animación
completa del proceso que se dispara por tópico o por **UDP** (celular / ESP32).

## Workspace

```
celda_ws/src/
├── celda_aceite/   (este paquete)
├── robot1/src/robot2/   R1  (paquete 'robot2')
├── robot2/              R2  (Scara_Final_description)
├── robot3/              R3  (urdfscarafinal)
├── robot4/              R4  (brazo_solidworks)
└── robot5/              R5  (modelo_robot)
```

```bash
cd ~/Documents/celda_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install          # debe decir: 6 packages finished
source install/setup.bash
ros2 launch celda_aceite celda.launch.py
```

Opciones del launch: `robots:=robot1,robot3` (solo algunos), `gui:=false` (sin ventana),
`coordinador:=false` (sin animación), `colisiones:=false` (los robots se atraviesan sin aviso; por defecto están ACTIVAS: los robots chocan físicamente entre sí y `monitor_choques.py` avisa cada choque en el log del launch como `CHOQUE robotA/eslabón <-> robotB/eslabón` y en el tópico `/celda/choques`).

Ver los choques desde otra terminal:

```bash
ros2 topic echo /celda/choques
```

## Disparar la animación

Espera unos segundos a que los 5 robots tengan sus controladores activos y manda **START**:

| Desde | Cómo |
|---|---|
| Terminal | `ros2 topic pub --once /celda/comando std_msgs/msg/String "{data: START}"` |
| Celular / ESP32 | Texto `START` por **UDP** a la IP de la PC, **puerto 5005** |

Comandos:

| Comando | Qué hace |
|---|---|
| `START` | Arranca la **línea de producción** (solo animación): botellas sin parar, varios robots a la vez |
| `UNA` | Produce una sola botella (solo animación) |
| `REAL` | La línea con los **robots físicos**: cada ESP32 conectado hace su tramo y Gazebo lo acompaña |
| `UNA REAL` | Una sola botella con los robots físicos |
| `STOP` | Deja de tomar envases nuevos, termina los que están en curso y los robots vuelven a HOME |
| `ESTADO` | LIBRE / PRODUCIENDO y el conteo |
| `HOME` / `RESET` | Robots a HOME (`RESET` además repone botella y tapa) |

Por UDP el coordinador contesta cada comando. El paso actual se publica en `/celda/estado` y la producción en
`/celda/produccion`:

```bash
ros2 topic echo /celda/produccion
```

## La línea

1. **R2** toma un envase vacío en EB y lo deja en PLL (en EB aparece otro: los envases son infinitos).
2. **R1** carga aceite caliente en RI y lo vierte en la botella en PLL (RE y PLL son el mismo punto; R1 se
   acercó 7 cm a PLL para alcanzar).
3. **R3** lleva la botella llena de PLL a PS.
4. **R4** toma una tapa en RT (aparece otra) y la rosca sobre la botella en PS.
5. **R5** lleva la botella tapada a PF, donde sale de la escena.

Los cinco robots trabajan **en paralelo** (cada uno en su hilo). Para que no choquen, el trabajo de cada robot
está dividido en tramos (ir / llevar / volver) y un tramo solo arranca si ningún otro robot está en un tramo
incompatible: la tabla `config/conflictos.json` la genera `tools/tabla_conflictos.py` con las envolventes de
colisión que usa Gazebo (1.5 cm de holgura entre dos robots que se mueven, 5 mm contra uno quieto).
Si cambias rutas o posturas en `config/secuencia.json`, regenera la tabla:

```bash
source install/setup.bash && python3 src/celda_aceite/tools/tabla_conflictos.py
```

Medido en Gazebo: **1 botella cada ~30.6 s ≈ 1.96 botellas/minuto** en régimen, sin choques entre robots.
Las botellas/minuto se miden en tiempo simulado (si la PC simula más lento, la animación se hace más lenta
con ella y el resultado no cambia).

Botellas y tapas son objetos *animados*: el coordinador los mueve con el servicio `set_pose` de Gazebo pegados
a la herramienta del robot que los lleva. Hay 6 copias de cada una (`pool` en `config/estaciones.yaml`) que se
reutilizan.

## Robots reales (ESP32 por WiFi) siguiendo a Gazebo

El PC es el **nodo maestro**: el coordinador decide qué robot se mueve y cuándo (la misma máquina de
estados y el mismo semáforo que evitan choques en la simulación). Cada robot real tiene un ESP32 en la misma
red WiFi que el PC; no necesita ROS, solo UDP. Por cada tramo, Gazebo anima la misma parte que hace el robot
real, y la línea solo sigue cuando el ESP32 levanta su **bandera** `LISTO`.

| Quién | Mensaje (texto UDP, puerto 5005 del PC) | Cuándo |
|---|---|---|
| ESP32 → PC | `R2 HOLA` | al encender (el PC guarda su IP y contesta `R2 BIENVENIDO`) |
| PC → ESP32 | `R2 HAZ ir 17` | le toca ese tramo (Gazebo lo anima a la vez) |
| ESP32 → PC | `R2 OK ir 17` | recibido (si no llega, el PC repite la orden cada 1 s) |
| ESP32 → PC | `R2 LISTO ir 17` | **bandera**: el robot real terminó ese tramo |

Tramos: `ir`, `llevar`, `volver` (R2 a R5) y `salir`, `cargar`, `verter`, `volver` (R1). El número
(`17`) es el número de orden, para que un mensaje repetido no se confunda con el ciclo siguiente.

Pasos:

1. Programa cada ESP32 con `tools/esp32_robot/esp32_robot.ino`: cambia `ROBOT`, el WiFi y la IP del PC, y
   pon el código de motores de cada tramo en `mover_tramo()` (debe volver cuando el robot terminó).
2. En el PC lanza la celda como siempre:

   ```bash
   ros2 launch celda_aceite celda.launch.py
   ```

3. Enciende los ESP32: en el log aparece `robotN: ESP32 conectado desde <IP>` por cada uno.
4. Manda **`REAL`** (en vez de `START`). Los robots con ESP32 conectado esperan su bandera; los que no
   tienen ESP32 solo se animan:

   ```bash
   ros2 topic pub --once /celda/comando std_msgs/msg/String "{data: REAL}"
   ```

   Si no hay ningún ESP32 conectado contesta `NO LISTO`. Para exigir que estén todos antes de arrancar:
   `ros2 launch celda_aceite celda.launch.py reales:=todos`.

Si un ESP32 no manda `LISTO` en 120 s (`espera_listo_s`), la línea se detiene con un error. El firewall del PC
debe dejar pasar UDP al puerto 5005.

Para probar sin hardware hay 5 ESP32 simulados:

```bash
python3 src/celda_aceite/tools/esp32_simulado.py 127.0.0.1 1,2,3,4,5 4
```

## Página web del robot 3

Al lanzar la celda se abre también una página para el R3 en **http://localhost:8080** (o
`http://<IP del PC>:8080` desde el celular en la misma red; `interfaz:=false` para no lanzarla):

- **Banderas** entre el PC y el ESP32 del R3 (`HAZ` / `OK` / `LISTO`) y el tramo en que está.
- **Ángulos** del R3 en Gazebo y de sus encoders, con la diferencia (en rojo si pasa de 3° o 3 mm).
- **Mover a mano** (solo con la línea detenida): deslizadores por junta y botones HOME / PLL / PS. Mueve el R3
  de Gazebo y el real a la vez (`MANUAL R3 e1 e2 z garra` al coordinador → `R3 MUEVE ...` al ESP32).
- **Abrir RViz**: el R3 de Gazebo (sólido) y el R3 real según sus encoders (semitransparente).

Para eso el ESP32 del R3 manda cada 100 ms `R3 POS e1 e2 z garra` (rad, rad, m, rad) y acepta
`R3 MUEVE e1 e2 z garra <n>` (contesta `R3 OK MUEVE <n>` y `R3 LISTO MUEVE <n>`); ya está en
`tools/esp32_robot/esp32_robot.ino` (`leer_angulos()` y `mover_articulaciones()`).

## Archivos

| Archivo | Para qué |
|---|---|
| `config/robots.yaml` | qué robot va dónde (centro de la base, giro) y su HOME |
| `config/estaciones.yaml` | estaciones y objetos → `python3 tools/generar_mundo.py` regenera el mundo |
| `config/secuencia.json` | rutas de cada robot (HOME → recoger → dejar → HOME) |
| `config/conflictos.json` | tramos que no pueden coincidir (lo genera `tools/tabla_conflictos.py`) |
| `scripts/coordinador.py` | la línea: máquina de estados por robot, semáforo, banderas con los ESP32 |
| `scripts/monitor_choques.py` | avisa choques entre robots en Gazebo |
| `scripts/interfaz_r3.py` | página web del R3 |
| `tools/esp32_robot/esp32_robot.ino` | programa base de cada ESP32 |
| `tools/esp32_simulado.py` | ESP32 simulados para probar sin hardware |
| `tools/geometria_celda.py`, `tabla_conflictos.py`, `replanear_tramo.py` | choques y rutas fuera de línea |
| `models/` | botella vacía, botella llena, tapa, dispensador de tapas |

## Notas

- Los ejes verticales (corredera del R2, Z del R3, R4 y R5) nunca se mandan justo a su tope: quedan 1 mm
  antes, porque en Gazebo una junta en su tope a veces se traba.
- R5 está girado −90° porque sus articulaciones solo giran ±90°: así alcanza PS y PF.
- Pendiente: el dedo de la garra del R3 roza el brazo 1 del R4 (en HOME) al llegar a PS. Gazebo no lo ve
  porque los dedos no tienen colisión.
