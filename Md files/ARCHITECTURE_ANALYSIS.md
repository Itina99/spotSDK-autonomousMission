# Analisi Comparativa: easy_walk.py vs easy_walk_ros.py

## Indice

1. [Introduzione](#introduzione)
2. [Funzioni Identiche](#funzioni-identiche)
3. [Funzioni che Richiedono Adattamento](#funzioni-che-richiedono-adattamento)
4. [Cosa è Diverso](#cosa-è-diverso)
5. [Elenco delle Funzioni](#elenco-delle-funzioni)
6. [Passi per l'Implementazione](#passi-per-limplementazione)

---

## Introduzione

Questo documento analizza le differenze tra le due versioni dell'algoritmo di esplorazione:
- **easy_walk.py**: Versione per Boston Dynamics Spot SDK
- **easy_walk_ros.py**: Versione per ROS2 + Gazebo

Lo scopo è identificare cosa può essere centralizzato (algoritmo puro) e cosa deve essere adattato tramite **adapter pattern** per supportare entrambi i backend.

---

## ✅ Funzioni Identiche o Quasi Identiche

Queste funzioni possono essere **centralizzate in `algorithms/`** perché non dipendono da dettagli hardware specifici.

### Tabella Riepilogativa

| Funzione | SDK | ROS | Status |
|----------|-----|-----|--------|
| `sample_cell_points()` | Linee 89-130 | Linee 104-128 | **IDENTICA** (solo np.cos vs math.cos) |
| `find_best_point_in_cell()` | Linee 133-189 | Linee 131-182 | **MOLTO SIMILE** (ROS aggiunge `local_distance` parameter) |
| `draw_explored_sides()` | Linee 192-250 | Linee 185-208 | **SIMILE** (SDK usa matplotlib, ROS raccoglie segmenti) |
| `find_new_borders()` | Linee 814-821 | Linee 1165-1172 | **IDENTICA** |
| `check_line_of_sight()` core logic | Linee 46-86 | Linee 36-72 | **CORE IDENTICO** (ROS aggiunge path per OccupancyGridHelper) |
| Loop principale exploration | Linee 916-1031 | Linee 1343-1422 | **STRUTTURA IDENTICA** (diverse solo le chiamate SDK/ROS) |

### Dettagli per Funzione

#### `sample_cell_points()`

**SDK (linee 89-130):**
```python
def sample_cell_points(env, cell_row, cell_col, num_samples=200):
    world_pos = env.get_world_position_from_cell(cell_row, cell_col)
    if world_pos is None:
        return []
    cell_center_x, cell_center_y = world_pos
    half_size = env.cell_size / 2.0
    samples = []
    for _ in range(num_samples):
        offset_x = np.random.uniform(-half_size * 0.8, half_size * 0.8)
        offset_y = np.random.uniform(-half_size * 0.8, half_size * 0.8)
        cos_yaw = np.cos(env.origin_yaw)
        sin_yaw = np.sin(env.origin_yaw)
        # ... rest of logic
    return samples
```

**ROS (linee 104-128):**
```python
def sample_cell_points(env, cell_row, cell_col, num_samples=200):
    world_pos = env.get_world_position_from_cell(cell_row, cell_col)
    # ... identical logic, only math.cos vs np.cos
```

**Conclusione**: La logica è identica. L'unica differenza è l'uso di `math.cos` vs `np.cos`. Centralizzare in `algorithms/utils/cell_sampling.py` usando `np` (più coerente).

#### `find_new_borders()`

**SDK (linee 814-821):**
```python
def find_new_borders(env, robot_row, robot_col, path, frontier):
    new_borders = env.get_adjacent_frontier_cells(robot_row, robot_col, path)
    new_borders_cells = []
    if len(new_borders) != 0:
        for new_border in new_borders:
            if new_border not in frontier and env.is_cell_visited(new_border[0], new_border[1]) != 1:
                new_borders_cells.append(new_border)
    return new_borders_cells
```

**ROS (linee 1165-1172):**
```python
def find_new_borders(env, robot_row, robot_col, path, frontier):
    # Identica
```

**Conclusione**: Completamente identica. Centralizzare in `algorithms/exploration_utils.py`.

---

## 🔄 Funzioni che Richiedono Adattamento via Adapter

Queste funzioni accedono a risorse hardware-specifiche e richiedono un layer di astrazione.

### 1. ACCESS A OBSTACLE DISTANCE GRID

Questa è la **differenza più significativa** nell'algoritmo.

#### SDK Version (linee 703-704, 732-733)

```python
proto = local_grid_client.get_local_grids(['obstacle_distance'])
pts, cells_obstacle_dist, color = spotGrid.create_vtk_obstacle_grid(proto, robot_state_client)
```

**Dettagli SDK:**
- Accede alla **LocalGridClient** di Bosdyn
- Richiede il **robot_state_client** per estrarre le trasformazioni
- Ritorna: `(pts, cells_obstacle_dist, color)` - array numpy

#### ROS Version (linee 1079-1083)

```python
helper, pts, cells_obstacle_dist = spotGrid_ros.create_obstacle_grid_from_occupancy(
    node.current_map,
    occupied_threshold=occupied_threshold,
    treat_unknown_as_obstacle=False,
)
```

**Dettagli ROS:**
- Accede all'**OccupancyGrid** ricevuto da SLAM/LIDAR
- Ritorna: `(helper, pts, cells_obstacle_dist)` - helper è un OccupancyGridHelper
- ROS ha anche `node.local_distance` (SDF statica da file) come backup

#### 🎯 Adapter Richiesto

```python
from abc import ABC, abstractmethod
import numpy as np
from typing import Tuple

class LocalGridProvider(ABC):
    """Interfaccia per accedere ai dati della griglia locale di ostacoli."""
    
    @abstractmethod
    def get_obstacle_distance_grid(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Ritorna (pts, cells_obstacle_dist, color)
        
        Returns:
            pts: Array N x 2 con coordinate (x, y) di ogni punto
            cells_obstacle_dist: Array N con distanza da ostacoli per ogni punto
            color: Array N x 3 con colori per visualizzazione
        """
        pass


# Implementazione SDK
class SDKLocalGridProvider(LocalGridProvider):
    def __init__(self, local_grid_client, robot_state_client):
        self.local_grid_client = local_grid_client
        self.robot_state_client = robot_state_client
    
    def get_obstacle_distance_grid(self):
        proto = self.local_grid_client.get_local_grids(['obstacle_distance'])
        pts, cells_obstacle_dist, color = spotGrid.create_vtk_obstacle_grid(
            proto, self.robot_state_client
        )
        return pts, cells_obstacle_dist, color


# Implementazione ROS
class ROSLocalGridProvider(LocalGridProvider):
    def __init__(self, current_map, occupied_threshold=65):
        self.current_map = current_map
        self.occupied_threshold = occupied_threshold
    
    def get_obstacle_distance_grid(self):
        helper, pts, cells = spotGrid_ros.create_obstacle_grid_from_occupancy(
            self.current_map,
            occupied_threshold=self.occupied_threshold,
            treat_unknown_as_obstacle=False
        )
        # ROS non ritorna color come array numpy, lo creiamo vuoto
        color = np.zeros((len(pts), 3), dtype=np.uint8)
        return pts, cells, color
```

---

### 2. ROBOT POSITION

#### SDK Version (linee 720-726)

```python
vision_tform_body = get_a_tform_b(
    transforms_snapshot,
    VISION_FRAME_NAME,
    BODY_FRAME_NAME
)
robot_x = vision_tform_body.position.x
robot_y = vision_tform_body.position.y
```

**Dettagli SDK:**
- Usa **frame transforms** di Bosdyn
- Richiede `VISION_FRAME_NAME` e `BODY_FRAME_NAME` (costanti SDK)
- Ritorna SE3Pose con posizione e rotazione

#### ROS Version (linee 1086)

```python
robot_x, robot_y, _, _ = spotUtils_ros.getPosition(node.pose_state)
```

**Dettagli ROS:**
- Accede a **node.pose_state** (aggiornato da odometry)
- Ritorna direttamente coordinate (x, y, z, yaw)

#### 🎯 Adapter Richiesto

```python
from abc import ABC, abstractmethod
from typing import Tuple, Any

class StateProvider(ABC):
    """Interfaccia per accedere allo stato del robot."""
    
    @abstractmethod
    def get_position(self) -> Tuple[float, float, float]:
        """Ritorna (x, y, z) in coordinates assolute."""
        pass
    
    @abstractmethod
    def get_yaw(self) -> float:
        """Ritorna l'angolo yaw in radianti."""
        pass
    
    @abstractmethod
    def get_quaternion(self) -> Any:
        """Ritorna il quaternione di rotazione (formato nativo del sistema)."""
        pass


# Implementazione SDK
class SDKStateProvider(StateProvider):
    def __init__(self, robot_state_client, transforms_snapshot):
        self.robot_state_client = robot_state_client
        self.transforms_snapshot = transforms_snapshot
    
    def get_position(self):
        vision_tform_body = get_a_tform_b(
            self.transforms_snapshot,
            VISION_FRAME_NAME,
            BODY_FRAME_NAME
        )
        return (vision_tform_body.position.x, 
                vision_tform_body.position.y, 
                vision_tform_body.position.z)
    
    def get_yaw(self):
        vision_tform_body = get_a_tform_b(
            self.transforms_snapshot,
            VISION_FRAME_NAME,
            BODY_FRAME_NAME
        )
        quat = vision_tform_body.rotation
        return np.arctan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                         1.0 - 2.0 * (quat.y**2 + quat.z**2))
    
    def get_quaternion(self):
        vision_tform_body = get_a_tform_b(
            self.transforms_snapshot,
            VISION_FRAME_NAME,
            BODY_FRAME_NAME
        )
        return vision_tform_body.rotation


# Implementazione ROS
class ROSStateProvider(StateProvider):
    def __init__(self, pose_state):
        self.pose_state = pose_state
    
    def get_position(self):
        return (self.pose_state.x, self.pose_state.y, self.pose_state.z)
    
    def get_yaw(self):
        return self.pose_state.yaw()
    
    def get_quaternion(self):
        return None  # ROS non memorizza quaternione nel pose_state
```

---

### 3. MOVEMENT COMMANDS

#### SDK Version (linee 784-793)

```python
success_rot = movements.relative_move(0, 0, dyaw, "vision",
                                     command_client, robot_state_client)
time.sleep(0.5)
success_move = movements.relative_move(distance, 0, 0, "vision",
                                      command_client, robot_state_client)
```

**Dettagli SDK:**
- `movements.relative_move()` accetta (fwd, strafe, turn, frame, client, state_client)
- Richiede `time.sleep()` tra comandi
- Ritorna bool per successo/fallimento

#### ROS Version (linee 1135-1139)

```python
if not node.motion.rotate_by(dyaw):
    return False
if not node.motion.move_to(target_x, target_y):
    return False
```

**Dettagli ROS:**
- `node.motion` è un **MotionController** con metodi di alto livello
- `rotate_by(dyaw)`: ruota di un angolo
- `move_to(x, y)`: si muove verso coordinata assoluta

#### 🎯 Adapter Richiesto

```python
from abc import ABC, abstractmethod

class MovementProvider(ABC):
    """Interfaccia per controllare i movimenti del robot."""
    
    @abstractmethod
    def rotate_by(self, dyaw: float) -> bool:
        """Ruota di un angolo [rad]."""
        pass
    
    @abstractmethod
    def move_forward(self, distance: float) -> bool:
        """Muove in avanti di una distanza [m]."""
        pass
    
    @abstractmethod
    def move_to(self, target_x: float, target_y: float) -> bool:
        """Muove verso coordinate assolute (x, y)."""
        pass


# Implementazione SDK
class SDKMovementProvider(MovementProvider):
    def __init__(self, command_client, robot_state_client):
        self.command_client = command_client
        self.robot_state_client = robot_state_client
    
    def rotate_by(self, dyaw: float) -> bool:
        return movements.relative_move(0, 0, dyaw, "vision",
                                       self.command_client, 
                                       self.robot_state_client)
    
    def move_forward(self, distance: float) -> bool:
        return movements.relative_move(distance, 0, 0, "vision",
                                       self.command_client, 
                                       self.robot_state_client)
    
    def move_to(self, target_x: float, target_y: float) -> bool:
        # SDK non ha move_to(), deve essere implementato combinando
        # rotazione + movimento
        # Questo dovrebbe essere fatto in explore loop, non qui
        raise NotImplementedError("SDK movement provider deve usare move_forward/rotate_by")


# Implementazione ROS
class ROSMovementProvider(MovementProvider):
    def __init__(self, motion_controller):
        self.motion = motion_controller
    
    def rotate_by(self, dyaw: float) -> bool:
        return self.motion.rotate_by(dyaw)
    
    def move_forward(self, distance: float) -> bool:
        # ROS motion controller ha move_to(), non move_forward()
        # Devo calcolare target_x, target_y dalle coordinate del robot
        raise NotImplementedError("ROS usa move_to(), non move_forward()")
    
    def move_to(self, target_x: float, target_y: float) -> bool:
        return self.motion.move_to(target_x, target_y)
```

---

### 4. VISUALIZATION

#### SDK Version (linee 740-770)

```python
visualize_grid_with_candidates(
    pts, cells_obstacle_dist, color, robot_x, robot_y,
    {'rejected': rejected_samples, 'valid': valid_samples},
    (target_x, target_y), iteration, env, save_path
)
```

**Dettagli SDK:**
- Crea figura matplotlib
- Salva PNG su disco
- Visualizza contemporaneamente sullo schermo

#### ROS Version (linee 1099-1125)

```python
visualize_grid_static(
    node.local_distance,
    robot_x, robot_y,
    {'rejected': rejected_samples, 'valid': []},
    None,
    iteration,
    env=env
)
```

**Dettagli ROS:**
- Pubblica marker di RViz
- Niente file su disco (streaming in tempo reale)
- Due visualizer: `visualize_grid_with_candidates()` e `visualize_grid_static()`

#### 🎯 Adapter Richiesto

```python
class VisualizerProvider(ABC):
    """Interfaccia per visualizzazione durante esplorazione."""
    
    @abstractmethod
    def visualize_iteration(
        self,
        pts: np.ndarray,
        cells_obstacle_dist: np.ndarray,
        robot_x: float,
        robot_y: float,
        candidates: dict,
        chosen_point: Tuple[float, float],
        iteration: int,
        env: 'EnvironmentMap',
    ) -> None:
        """Visualizza lo stato dell'esplorazione."""
        pass


# Implementazione SDK
class SDKVisualizerProvider(VisualizerProvider):
    def __init__(self, mission_folder=None):
        self.mission_folder = mission_folder
    
    def visualize_iteration(self, pts, cells_obstacle_dist, robot_x, robot_y,
                           candidates, chosen_point, iteration, env):
        save_path = None
        if self.mission_folder:
            save_path = os.path.join(
                self.mission_folder,
                f"iteration_{iteration}.png"
            )
        visualize_grid_with_candidates(
            pts, cells_obstacle_dist, None, robot_x, robot_y,
            candidates, chosen_point, iteration, env, save_path
        )


# Implementazione ROS
class ROSVisualizerProvider(VisualizerProvider):
    def __init__(self, node, local_distance=None):
        self.node = node
        self.local_distance = local_distance
    
    def visualize_iteration(self, pts, cells_obstacle_dist, robot_x, robot_y,
                           candidates, chosen_point, iteration, env):
        visualize_grid_static(
            self.local_distance,
            robot_x, robot_y,
            candidates, chosen_point, iteration, env
        )
```

---

### 5. WAYPOINT & RECORDING MANAGEMENT

#### SDK Version (linee 949-950)

```python
recordingInterface.create_default_waypoint(
    cell_row=selected_border[0],
    cell_col=selected_border[1]
)
```

**Dettagli SDK:**
- `RecordingInterface` di navGraphUtils
- Crea waypoints nel navgraph di Spot
- Gestisce il graph-nav backend

#### ROS Version (linee 1372-1379)

```python
recording.create_default_waypoint(
    cell_row=selected_border[0],
    cell_col=selected_border[1],
    x=x_new, y=y_new, z=z_new, yaw=node.pose_state.yaw(),
)
```

**Dettagli ROS:**
- `navGraphUtils_ros.RecordingInterface`
- Più semplice, memorizza waypoints in memoria
- Richiede coordinata (x, y, z, yaw) esplicita

#### 🎯 Adapter Richiesto

```python
class RecordingProvider(ABC):
    """Interfaccia per gestione waypoint e map recording."""
    
    @abstractmethod
    def create_waypoint(self, cell_row: int, cell_col: int, 
                       x: float, y: float, z: float, yaw: float) -> None:
        """Crea un waypoint."""
        pass
    
    @abstractmethod
    def get_all_waypoints(self) -> dict:
        """Ritorna tutti i waypoint mappati per cella."""
        pass
    
    @abstractmethod
    def find_nearest_waypoint_to_target(self, 
                                       target_cell: Tuple[int, int]) -> Tuple[int, int]:
        """Trova waypoint più vicino al target."""
        pass


# Implementazione SDK
class SDKRecordingProvider(RecordingProvider):
    def __init__(self, recording_interface):
        self.recording_interface = recording_interface
    
    def create_waypoint(self, cell_row, cell_col, x, y, z, yaw):
        # SDK non richiede x, y, z - li estrae da transform
        self.recording_interface.create_default_waypoint(
            cell_row=cell_row,
            cell_col=cell_col
        )
    
    def get_all_waypoints(self):
        return self.recording_interface.get_all_manual_waypoints_with_cells()
    
    def find_nearest_waypoint_to_target(self, target_cell):
        waypoints = self.get_all_waypoints()
        return self.recording_interface.find_nearest_waypoint_cell_to_target(
            target_cell, waypoints, env=None
        )


# Implementazione ROS
class ROSRecordingProvider(RecordingProvider):
    def __init__(self, recording_interface):
        self.recording_interface = recording_interface
    
    def create_waypoint(self, cell_row, cell_col, x, y, z, yaw):
        self.recording_interface.create_default_waypoint(
            cell_row=cell_row,
            cell_col=cell_col,
            x=x, y=y, z=z, yaw=yaw
        )
    
    def get_all_waypoints(self):
        return self.recording_interface.get_all_manual_waypoints_with_cells()
    
    def find_nearest_waypoint_to_target(self, target_cell):
        waypoints = self.get_all_waypoints()
        return self.recording_interface.find_nearest_waypoint_cell_to_target(
            target_cell, waypoints, env=None
        )
```

---

## ❌ Cosa è Diverso

Queste differenze **non possono essere astatte** e rimangono specifiche per SDK/ROS.

### Tabella Riepilogativa

| Aspetto | SDK | ROS | Motivo |
|---------|-----|-----|--------|
| **Inizializzazione robot** | `setLogInfo()` con Bosdyn | `rclpy.init()` con Node | Framework diversi |
| **E-stop & Power** | `SimpleEstop`, `power_on()`, `power_off()` | N/A (Gazebo non ha stato di potenza) | Hardware vs Simulazione |
| **Setup File Logging** | Crea cartelle per missione con timestamp | N/A (rclpy logging standard) | SDK feature specific |
| **Graph-nav recording** | `RecordingInterface` con API Bosdyn | `navGraphUtils_ros.RecordingInterface` semplice | API fundamentalmente diversi |
| **Fiducial initialization** | `initialize_with_fiducial()` per SetOrigin | N/A | Spot-specific feature |
| **Graph optimization** | `auto_close_loops()`, `optimize_anchoring()` | N/A | Spot graph-nav backend |
| **Navigate to waypoint** | `navigate_to_waypoint()` via graph-nav | `node.motion.move_to()` semplice | Architetture diverse |

### Dettagli

#### Inizializzazione (SDK linee 823-846)

```python
def easy_walk(options):
    robot, lease_client, robot_state_client, client_metadata = spotLogInUtils.setLogInfo(options)
    estop = spotLogInUtils.SimpleEstop(robot, options.name + "_estop")
    
    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        local_grid_client = robot.ensure_client(LocalGridClient.default_service_name)
        robot.time_sync.wait_for_sync()
        robot.logger.info('Powering on robot...')
        robot.power_on()
        assert robot.is_powered_on(), 'Robot power on failed.'
        robot.logger.info('Robot powered on.')
        blocking_stand(command_client)
```

**Non può essere centralizzato** perché:
- Richiede SDK Bosdyn specifico
- Gestisce lease (concorrenza del robot)
- Richiede time_sync
- Power on/off è hardware-specific

#### Inizializzazione (ROS linee 1306-1325)

```python
def easy_walk(options=None):
    rclpy.init()
    node = EasyWalkROSNode()
    global ROS_NODE
    ROS_NODE = node
    
    try:
        if not node.wait_for_data():
            node.get_logger().error('Timeout waiting for map data.')
            return False
```

**Completamente diverso** - non condivisibile.

---

## 📋 Elenco delle Funzioni

### Centralizzare in `algorithms/exploration.py`

Queste funzioni formano il **cuore dell'algoritmo** e sono indipendenti da hardware:

```
✅ sample_cell_points(env, cell_row, cell_col, num_samples)
✅ find_best_point_in_cell(robot_x, robot_y, env, cell_row, cell_col, ...)
✅ find_new_borders(env, robot_row, robot_col, path, frontier)
✅ check_line_of_sight(x1, y1, x2, y2, pts, cells, threshold)
✅ exploration_main_loop(providers, env, path)
```

### In `algorithms/utils/`

```
algorithms/utils/
├── line_of_sight.py
│   └── check_line_of_sight()
│
├── cell_sampling.py
│   └── sample_cell_points()
│
├── exploration_utils.py
│   ├── find_new_borders()
│   ├── find_best_point_in_cell()
│   └── constants/thresholds
│
└── visualization_base.py
    └── Base visualization logic (no matplotlib/RViz)
```

### In `core/interfaces.py`

Provider astrazioni (pattern Adapter):

```python
LocalGridProvider (ABC)
StateProvider (ABC)
MovementProvider (ABC)
VisualizerProvider (ABC)
RecordingProvider (ABC)
```

### In `spot/adapters.py`

Implementazioni Spot SDK:

```python
class SDKLocalGridProvider(LocalGridProvider)
class SDKStateProvider(StateProvider)
class SDKMovementProvider(MovementProvider)
class SDKVisualizerProvider(VisualizerProvider)
class SDKRecordingProvider(RecordingProvider)
```

### In `spot_ros/adapters.py`

Implementazioni ROS2:

```python
class ROSLocalGridProvider(LocalGridProvider)
class ROSStateProvider(StateProvider)
class ROSMovementProvider(MovementProvider)
class ROSVisualizerProvider(VisualizerProvider)
class ROSRecordingProvider(RecordingProvider)
```

### Entry Points

**`entry_points/run_exploration_sdk.py`**
```python
def main():
    # Setup SDK
    robot, lease_client, ... = spotLogInUtils.setLogInfo(options)
    
    # Crea adapters
    local_grid_provider = SDKLocalGridProvider(...)
    state_provider = SDKStateProvider(...)
    movement_provider = SDKMovementProvider(...)
    # ...
    
    # Chiama algoritmo centralizzato
    exploration_main_loop(
        providers={'local_grid': local_grid_provider, ...},
        env=env,
        path=path
    )
```

**`entry_points/run_exploration_ros.py`**
```python
class EasyWalkROSNode(Node):
    def explore(self):
        # Setup ROS
        local_grid_provider = ROSLocalGridProvider(self.current_map)
        state_provider = ROSStateProvider(self.pose_state)
        # ...
        
        # Chiama algoritmo centralizzato
        exploration_main_loop(
            providers={'local_grid': local_grid_provider, ...},
            env=env,
            path=path
        )
```

---

## 🎯 Passi per l'Implementazione

### Passo 1: Estrai Funzioni Pure

**Da `easy_walk.py` → `algorithms/utils/`**

1. Copia `check_line_of_sight()` in `algorithms/utils/line_of_sight.py`
2. Copia `sample_cell_points()` in `algorithms/utils/cell_sampling.py`
3. Copia `find_best_point_in_cell()` in `algorithms/utils/exploration_utils.py`
4. Copia `find_new_borders()` in `algorithms/utils/exploration_utils.py`

**Attenzione**: Rimuovi dipendenze SDK (bosdyn import, spotGrid calls)

### Passo 2: Crea Abstrazioni Provider

**File: `core/interfaces.py`**

1. Scrivi `LocalGridProvider` (ABC)
2. Scrivi `StateProvider` (ABC)
3. Scrivi `MovementProvider` (ABC)
4. Scrivi `VisualizerProvider` (ABC)
5. Scrivi `RecordingProvider` (ABC)

### Passo 3: Implementa Adapters SDK

**File: `spot/adapters.py`**

1. Crea `SDKLocalGridProvider(LocalGridProvider)`
2. Crea `SDKStateProvider(StateProvider)`
3. Crea `SDKMovementProvider(MovementProvider)`
4. Crea `SDKVisualizerProvider(VisualizerProvider)`
5. Crea `SDKRecordingProvider(RecordingProvider)`

### Passo 4: Implementa Adapters ROS

**File: `spot_ros/adapters.py`**

1. Crea `ROSLocalGridProvider(LocalGridProvider)`
2. Crea `ROSStateProvider(StateProvider)`
3. Crea `ROSMovementProvider(MovementProvider)`
4. Crea `ROSVisualizerProvider(VisualizerProvider)`
5. Crea `ROSRecordingProvider(RecordingProvider)`

### Passo 5: Scrivi Algoritmo Centralizzato

**File: `algorithms/exploration.py`**

1. Aggiungi `exploration_main_loop(providers, env, path)`
2. Questa funzione riceve tutti i provider come parametri
3. Chiama le funzioni pure da `algorithms/utils/`
4. Usa provider per interagire con hardware

Esempio di signature:

```python
def exploration_main_loop(
    providers: Dict[str, Any],
    env: EnvironmentMap,
    path: List[Tuple[int, int]],
    max_iterations: int = 1000,
) -> bool:
    """
    Main exploration loop using dependency injection.
    
    Args:
        providers: Dict con chiavi 'local_grid', 'state', 'movement', 'visualizer', 'recording'
        env: EnvironmentMap instance
        path: Serpentine path
        max_iterations: Max iterations
    
    Returns:
        bool: True se esplorazione completata
    """
    local_grid_provider = providers['local_grid']
    state_provider = providers['state']
    movement_provider = providers['movement']
    visualizer = providers['visualizer']
    recording = providers['recording']
    
    frontier = []
    iteration = 0
    
    # Get initial position
    robot_x, robot_y, _, _ = state_provider.get_position()
    # ... rest of exploration loop
```

### Passo 6: Scrivi Entry Points

**File: `entry_points/run_exploration_sdk.py`**
- Setup SDK
- Crea adapters SDK
- Chiama `exploration_main_loop()`

**File: `entry_points/run_exploration_ros.py`**
- Setup ROS2 Node
- Crea adapters ROS
- Chiama `exploration_main_loop()` da dentro il node

### Passo 7: Elimina Duplicazione

1. **Delete** `easy_walk_ros.py` (logica ora in `algorithms/exploration.py`)
2. **Keep** `easy_walk.py` come entry point SDK legacy (o equivalente in `entry_points/`)
3. **Refactor** `easy_walk.py` per usare i nuovi adapter/centralizzato

### Passo 8: Test & Validazione

1. Test suite per funzioni pure in `algorithms/`
2. Mock provider per unit test
3. Test end-to-end SDK
4. Test end-to-end ROS

---

## 📦 Gestione degli Import: Strategia e Best Practices

### 🎯 Principio Fondamentale

L'algoritmo **NON deve avere import condizionati** basati sulla piattaforma. Invece:

1. **`algorithms/exploration.py`** importa SOLO:
   - Librerie standard (`numpy`, `typing`)
   - Le funzioni pure da `algorithms/utils/`
   - **Le astrazioni** da `core/interfaces.py` (non le implementazioni!)

2. **I provider concreti** (SDK vs ROS) vengono **iniettati** tramite dependency injection

3. **Gli import specializzati** (bosdyn, rclpy) rimangono **solo negli adapter**

---

### 📋 Import per Ogni File

#### **`algorithms/exploration.py`** (Puro - NO import bosdyn/rclpy)

```python
# Standard library
from typing import Dict, List, Tuple, Any, Optional
import numpy as np

# Internal imports - utilities pure
from algorithms.utils.line_of_sight import check_line_of_sight
from algorithms.utils.cell_sampling import sample_cell_points
from algorithms.utils.exploration_utils import find_new_borders, find_best_point_in_cell

# Internal imports - interfaces (abstrazioni, non implementazioni!)
from core.interfaces import (
    LocalGridProvider,
    StateProvider,
    MovementProvider,
    VisualizerProvider,
    RecordingProvider,
)

# Altre dipendenze pure
from environmentMap import EnvironmentMap  # Spostare poi in algorithms/
```

**❌ MAI questi import:**
```python
import bosdyn.client  # ❌ NO
import rclpy          # ❌ NO
from spot import movements  # ❌ NO (solo nelle implementazioni concrete)
from spot_ros import movements_ros  # ❌ NO (solo nelle implementazioni concrete)
```

---

#### **`algorithms/utils/line_of_sight.py`**

```python
from typing import List, Tuple
import numpy as np
```

Assolutamente niente platform-specific.

---

#### **`algorithms/utils/cell_sampling.py`**

```python
from typing import List, Tuple
import numpy as np
```

Puro.

---

#### **`algorithms/utils/exploration_utils.py`**

```python
from typing import List, Tuple, Optional
import numpy as np

# Solo EnvironmentMap - che è pura
from environmentMap import EnvironmentMap
```

Puro.

---

#### **`core/interfaces.py`** (Astrazioni)

```python
from abc import ABC, abstractmethod
from typing import Tuple, Dict, Any, Optional
import numpy as np
```

Niente di platform-specific - solo astrazioni.

---

#### **`spot/adapters.py`** (Implementazioni SDK)

```python
# Standard
from typing import Tuple, Dict, Any
import numpy as np

# Internal interfaces
from core.interfaces import (
    LocalGridProvider,
    StateProvider,
    MovementProvider,
    VisualizerProvider,
    RecordingProvider,
)

# SDK SPECIFICI - QUI SI!
import bosdyn.client
from bosdyn.api import basic_pb2
from bosdyn.client.frame_helpers import get_a_tform_b, VISION_FRAME_NAME, BODY_FRAME_NAME

# Internal SDK modules
from spot import movements
from spot import spotGrid
from spot import spotUtils
from spot import navGraphUtils
from spot import spotLogInUtils
```

**Solo qui gli import bosdyn!**

---

#### **`spot_ros/adapters.py`** (Implementazioni ROS)

```python
# Standard
from typing import Tuple, Dict, Any, Optional
import numpy as np

# Internal interfaces
from core.interfaces import (
    LocalGridProvider,
    StateProvider,
    MovementProvider,
    VisualizerProvider,
    RecordingProvider,
)

# ROS SPECIFICI - QUI SI!
import rclpy
from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import OccupancyGrid

# Internal ROS modules
from spot_ros import movements_ros
from spot_ros import spotGrid_ros
from spot_ros import spotUtils_ros
from spot_ros import navGraphUtils_ros
from spot_ros.ObstacleGrid import OccupancyGridHelper
```

**Solo qui gli import rclpy!**

---

### ⚡ Pattern: Dependency Injection vs Conditional Import

#### ❌ **SBAGLIATO** (Conditional Import)

```python
# algorithms/exploration.py - SBAGLIATO!
import sys

if 'ros' in sys.modules:
    from spot_ros import spotGrid_ros as spotGrid
else:
    from spot import spotGrid

def exploration_main_loop():
    # Ora spotGrid è condizionato...
    cells = spotGrid.create_obstacle_grid(...)  # Quale versione?
```

**Problemi:**
- ❌ Accoppiamento stretto
- ❌ Difficile da testare
- ❌ Non si sa quale versione viene usata finché non viene eseguito
- ❌ Logica algoritmica mescolata con dettagli platform

#### ✅ **CORRETTO** (Dependency Injection)

```python
# algorithms/exploration.py
from core.interfaces import LocalGridProvider

def exploration_main_loop(
    providers: Dict[str, LocalGridProvider],  # Provider ricevuto come parametro!
    env: EnvironmentMap,
):
    # Chiama provider - non sa e non importa quale implementazione
    pts, cells, color = providers['local_grid'].get_obstacle_distance_grid()
```

**Vantaggi:**
- ✅ Zero accoppiamento
- ✅ Facile da testare (passa mock provider)
- ✅ Chiaro quale versione viene usata
- ✅ Logica pura separata da dettagli platform

---

### 🔗 Grafico degli Import

```
algorithms/exploration.py (PURO)
    ├── imports numpy, typing
    ├── imports algorithms/utils/* (puro)
    ├── imports core/interfaces.py (astrazioni)
    └── ❌ MAI: bosdyn, rclpy, spot.*, spot_ros.*

core/interfaces.py (ASTRAZIONI)
    ├── imports abc, typing
    └── ❌ MAI: bosdyn, rclpy, spot.*, spot_ros.*

spot/adapters.py (SDK CONCRETO)
    ├── imports bosdyn.client ✅
    ├── imports core/interfaces.py ✅
    ├── imports spot/* ✅
    └── ❌ MAI: rclpy, spot_ros.*

spot_ros/adapters.py (ROS CONCRETO)
    ├── imports rclpy ✅
    ├── imports core/interfaces.py ✅
    ├── imports spot_ros/* ✅
    └── ❌ MAI: bosdyn, spot.*
```

---

### 🚀 Esempio Concreto: Flow di Esecuzione

#### **Scenario SDK**

```python
# entry_points/run_exploration_sdk.py
import bosdyn.client
from spot.adapters import (
    SDKLocalGridProvider,
    SDKStateProvider,
    SDKMovementProvider,
)
from algorithms.exploration import exploration_main_loop

def main():
    # Setup SDK
    robot = bosdyn.client.create_standard_sdk('MyRobot').create_robot('192.168.1.1')
    local_grid_client = robot.ensure_client(LocalGridClient.default_service_name)
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    
    # Crea adapters concreti SDK
    providers = {
        'local_grid': SDKLocalGridProvider(local_grid_client, robot_state_client),
        'state': SDKStateProvider(robot_state_client, transforms_snapshot),
        'movement': SDKMovementProvider(command_client, robot_state_client),
        # ... altri provider
    }
    
    # Chiama algoritmo PURO - senza sapere che usa SDK!
    exploration_main_loop(
        providers=providers,
        env=env,
        path=path,
    )
```

**Flow:**
1. ✅ SDK setup (solo qui)
2. ✅ Crea implementazioni concrete SDK (`SDKLocalGridProvider`, etc.)
3. ✅ Passa a `exploration_main_loop(providers=...)`
4. ✅ **exploration.py non sa che sta usando SDK!**

#### **Scenario ROS**

```python
# spot_ros/easy_walk_ros_node.py (ROS Node)
import rclpy
from spot_ros.adapters import (
    ROSLocalGridProvider,
    ROSStateProvider,
    ROSMovementProvider,
)
from algorithms.exploration import exploration_main_loop

class EasyWalkROSNode(rclpy.node.Node):
    def explore(self):
        # Crea adapters concreti ROS
        providers = {
            'local_grid': ROSLocalGridProvider(self.current_map),
            'state': ROSStateProvider(self.pose_state),
            'movement': ROSMovementProvider(self.motion),
            # ... altri provider
        }
        
        # Chiama algoritmo PURO - senza sapere che usa ROS!
        exploration_main_loop(
            providers=providers,
            env=env,
            path=path,
        )
```

**Flow:**
1. ✅ ROS2 setup (solo qui)
2. ✅ Crea implementazioni concrete ROS (`ROSLocalGridProvider`, etc.)
3. ✅ Passa a `exploration_main_loop(providers=...)`
4. ✅ **exploration.py non sa che sta usando ROS!**

---

### ✅ Checklist degli Import

```
✅ algorithms/exploration.py
   - numpy ✓
   - typing ✓
   - algorithms.utils.* ✓
   - core.interfaces ✓
   - EnvironmentMap ✓
   - ❌ bosdyn
   - ❌ rclpy
   - ❌ spot.*
   - ❌ spot_ros.*

✅ core/interfaces.py
   - abc ✓
   - typing ✓
   - numpy ✓
   - ❌ bosdyn
   - ❌ rclpy

✅ spot/adapters.py
   - bosdyn.* ✓
   - spot.* ✓
   - core.interfaces ✓
   - ❌ rclpy
   - ❌ spot_ros.*

✅ spot_ros/adapters.py
   - rclpy ✓
   - spot_ros.* ✓
   - core.interfaces ✓
   - ❌ bosdyn
   - ❌ spot.*
```

---
