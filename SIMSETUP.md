# Spot Sim: ROS 2 & Gazebo Ignition Setup

Questa guida ti accompagna passo passo nella simulazione del robot Boston Dynamics **Spot** con **ROS 2 Humble** e **Gazebo Ignition Fortress**.

## 1. Requisiti di Sistema
- **OS:** Ubuntu 22.04 LTS
- **ROS 2:** Humble
- **Simulatore:** Gazebo Ignition Fortress
- **Strumenti base:** `git`, `python3`, `colcon`

> Nota: se non hai ancora `colcon`, lo installerai nel passo 3.

---

## 2. Installa le dipendenze ROS + OpenCV
In un terminale **senza Conda attivo**, installa i pacchetti necessari:

```bash
sudo apt update
sudo apt install python3-opencv ros-humble-cv-bridge ros-humble-vision-msgs ros-humble-ros-gz ros-humble-pointcloud-to-laserscan
```

---

## 3. Crea il workspace e compila
1. Crea il workspace e clona il repository della simulazione:

```bash
mkdir -p ~/spot_sim_ws/src
cd ~/spot_sim_ws/src

# Repository ufficiale della simulazione
git clone https://github.com/g1y5x3/spot_gazebo_ros2.git
```

2. Torna nella root del workspace e compila:

```bash
cd ~/spot_sim_ws
colcon build --symlink-install
```

Se `colcon` non è installato:

```bash
sudo apt install python3-colcon-common-extensions
```

---

## 4. Source dell'ambiente
Ogni nuovo terminale deve conoscere ROS 2 e il workspace compilato.

```bash
source /opt/ros/humble/setup.bash
source ~/spot_sim_ws/install/setup.bash
```

Questo solo per me
```bash
source /opt/ros/humble/setup.bash
source /data/itina99/spot_sim_ws/install/setup.bash
```

Suggerimento: aggiungi queste righe in fondo a `~/.bashrc` per renderle permanenti.

---

## 5. Avvio della simulazione
### Avvio con mondo di default

```bash
ros2 launch spot_bringup spot.gazebo.launch.py
```

### Avvio con mondo personalizzato

```bash
ros2 launch spot_bringup spot.gazebo.launch.py world_file:=~/spot_sim_ws/percorso_al_file_sdf/custom_world.sdf
```

Nel mio caso
```bash
ros2 launch spot_bringup spot.gazebo.launch.py world_file:=/data/itina99/spot_sim_ws/worlds/test.sdf
```
---

## 6. Comandi di movimento (Hello World)
### Test rapido da terminale
Verifica che i controller ricevano input con una camminata in cerchio:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.4}}"
```

### Esecuzione script Python (ROS 2)
Abbiamo tradotto lo script "Hello Spot" in un nodo ROS 2 puro (`hello_spot_ros.py`).

1. Assicurati che Conda sia disattivato:

```bash
conda deactivate
```

2. Esegui il source degli ambienti (vedi punto 4).
3. Lancia lo script:

```bash
python3 hello_spot_ros.py
```

Lo script esegue una sequenza preimpostata:
- Rotazione sul posto
- Avanzamento dritto
- Scatto e salvataggio di un'immagine dalla telecamera termica simulata (`hello_spot_ros_image.jpg`)

---

## 7. Avvio completo Navigazione Autonoma (SLAM + TF Bridge + Algoritmo)
Per lanciare l'algoritmo completo di mappatura e navigazione servono 4 terminali.
In **ogni** terminale ricordati di disattivare Conda e fare il source:


In ogni terminale, prima fai:

```bash
conda deactivate
source /opt/ros/humble/setup.bash
source /data/itina99/spot_sim_ws/install/setup.bash
```

#### Terminale A: Gazebo
```bash
ros2 launch spot_bringup spot.gazebo.launch.py world_file:=/data/itina99/spot_sim_ws/worlds/test.sdf
```

#### Terminale B: Launch orchestrato (TF bridge + SLAM + RViz)
```bash
cd /data/itina99/Progetti/spotSDK-autonomousMission
ros2 launch launch_exploration.launch.py
```

#### Terminale C: Algoritmo di Esplorazione
```bash
cd /data/itina99/Progetti/spotSDK-autonomousMission
python3 -m spot_ros.easy_walk_ros --ros-args -p odom_topic:=/spot/odometry -p use_sim_time:=true
```

Nota: con questa modalità è normale che RViz non appaia immediatamente; viene avviato solo quando il nodo di attesa riceve la prima `OccupancyGrid` su `/map`.

Se resta in attesa troppo a lungo:
- verifica che Gazebo stia pubblicando `/spot/lidar/scan`
- verifica che SLAM Toolbox sia partito correttamente
- muovi Spot per qualche secondo per generare le prime celle di mappa

---

## 8. Nuovo Metodo: LocalGridService + Static Grid (Consigliato)

A partire dalla versione aggiornata, il sistema supporta un **LocalGridService** che pubblica una vista **locale ±2m** del mondo a partire dalla griglia statica del SDF. Questo offre:

- ✅ **Zero latenza**: griglia precomputata dal SDF
- ✅ **Visione locale realistica**: robot non vede oltre ±2m
- ✅ **Deterministica**: sempre lo stesso risultato
- ✅ **CPU-efficient**: estrae solo il ±2m rilevante

### Come lanciare con LocalGridService

#### Terminale A: Gazebo
```bash
conda deactivate
source /opt/ros/humble/setup.bash
source /data/itina99/spot_sim_ws/install/setup.bash
ros2 launch spot_bringup spot.gazebo.launch.py world_file:=/data/itina99/spot_sim_ws/worlds/test.sdf
```

#### Terminale B: LocalGridService (NUOVO ✨)
```bash
conda deactivate
source /opt/ros/humble/setup.bash
source /data/itina99/spot_sim_ws/install/setup.bash
cd /data/itina99/Progetti/spotSDK-autonomousMission
python3 -m spot_ros.local_grid_service
```



Questo nodo:
- Carica il SDF una sola volta
- Sottoscritto a `/odom` (posizione robot)
- Pubblica `/spot/local_grid` a 10 Hz (OccupancyGrid 30×30, ±2m)

#### Terminale C: Launch orchestrato (TF bridge + SLAM + RViz)
```bash
conda deactivate
source /opt/ros/humble/setup.bash
source /data/itina99/spot_sim_ws/install/setup.bash
cd /data/itina99/Progetti/spotSDK-autonomousMission
ros2 launch launch_exploration.launch.py
```

#### Terminale D: Algoritmo di Esplorazione (con Local Grid)
```bash
conda deactivate
source /opt/ros/humble/setup.bash
source /data/itina99/spot_sim_ws/install/setup.bash
cd /data/itina99/Progetti/spotSDK-autonomousMission
python3 -m spot_ros.easy_walk_ros --ros-args -p odom_topic:=/spot/odometry -p use_sim_time:=true
```

#### Comandi per GPU
```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia __VK_LAYER_NV_optimus=NVIDIA_only ros2 launch spot_bringup spot.gazebo.launch.py world_file:=/data/itina99/spot_sim_ws/worlds/test.sdf
```
```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia __VK_LAYER_NV_optimus=NVIDIA_only ros2 launch launch_exploration.launch.py
```


Il sistema automaticamente:
1. Sottoscritto a `/spot/local_grid` (se disponibile)
2. Usa la vista locale per checkare line-of-sight
3. Fallback al sistema precedente se il service non è attivo

### Confronto: Metodi di Pubblicazione Map

| Aspetto | Metodo Vecchio | LocalGridService |
|---------|---|---|
| **Fonte mappa** | SLAM real-time da sensori | SDF statico precomputato |
| **Latenza** | Variabile (sensori) | Zero (cache) |
| **FoV Robot** | Illimitato | ±2m (realistico) |
| **Terminali** | 3 (Gazebo + Launch + Algorithm) | 4 (+ LocalGridService) |
| **CPU** | Alto (SLAM) | Basso (lookup grid) |
| **Quando usare** | Testing rapido | Simulazione più realistica |

### Parametri LocalGridService

Se vuoi customizzare il service, modifica `local_grid_service.py`:

```python
self.local_range = 2.0           # Raggio visione ±X metri
self.local_grid_size = 30        # Celle della griglia locale (30×30)
sdf_path = "/path/to/test.sdf"   # Path SDF file
```
