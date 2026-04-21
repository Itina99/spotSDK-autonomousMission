# Algoritmo Dettagliato (ROS) - Run passo passo

Questo documento descrive in modo operativo come funziona **una run tipica** del nodo `spot_ros/easy_walk_ros.py`.

L'obiettivo e' farti seguire il comportamento reale del codice, dalla partenza fino al rientro, con tutte le decisioni importanti.

---

## 1) Obiettivo della run

Il nodo `EasyWalkROS` deve:

1. ricevere mappa e posa,
2. costruire/aggiornare una frontiera di celle da esplorare,
3. scegliere una cella target,
4. trovare un punto interno raggiungibile,
5. muoversi in sicurezza (evitamento ostacoli),
6. marcare la cella come visitata o bloccata,
7. ripetere finche' la frontiera e' vuota,
8. tornare al punto iniziale lungo il path registrato.

---

## 2) Sensori e dati usati durante la run

Durante l'esecuzione il nodo usa contemporaneamente questi stream:

- `/map` (`OccupancyGrid`): mappa globale (tipicamente da SLAM).
- `odom_topic` (default `/odom`, nel tuo setup spesso `/spot/odometry`): odometria robot.
- `local_obs_scan_topic` (default `/spot/lidar/scan`): osservazioni locali rapide.
- opzionale `camera_local_grid_topic` (default `/spot/camera/local_obstacles`): griglia locale da pipeline camere.

Dati interni principali:

- `env` (`EnvironmentMap`): griglia logica celle visitate/bloccate.
- `frontier`: lista celle candidate `(row, col, rank)`.
- `pose_state` / `pose_state_odom`: posa robot in map/odom.
- `self.current_map`: ultima occupancy grid ricevuta.
- `_local_obs_points_map`: ostacoli locali recenti da scan.

---

## 3) Run ipotetica completa (timeline reale)

Di seguito una run concreta simulata su griglia `5x5`, cella `2.0 m`.

## T0 - Avvio nodo

`main()`:

1. `rclpy.init()`
2. crea `EasyWalkROS()`
3. entra in `run()`

In `__init__` vengono creati:

- publisher: `cmd_vel`, `MarkerArray` per RViz,
- subscriber: map, odom, scan, camera-local-grid,
- controller di moto `movements_ros.MotionController`,
- parametri di sicurezza (soglie occupancy, avoidance, timeout, retry).

## T1 - Warm-up sensori

`run()` chiama `wait_for_data()`:

- finche' non arriva almeno una `/map`, il nodo aspetta.
- se scade timeout -> termina con errore.

Intanto:

- `_on_odom()` aggiorna posa odom e prova a trasformarla in map.
- `_on_map()` salva mappa e frame `map_frame`.
- `_on_scan()` popola osservazione locale ostacoli in frame map.

## T2 - Bootstrap missione

Alla prima partenza valida:

1. crea `env = EnvironmentMap(rows=5, cols=5, cell_size=2.0)`.
2. legge posa iniziale `(x_boot, y_boot, yaw_boot)`.
3. `env.set_origin(...)` definisce allineamento mondo<->celle.
4. aggiunge waypoint iniziale e posizione robot in `env`.
5. genera path serpentino (`generate_serpentine_path`).

## T3 - Prima frontiera

- converte posizione robot in cella `(robot_row, robot_col)`.
- prende adiacenti via `env.get_adjacent_frontier_cells(...)`.
- li inserisce in `frontier` con `_merge_frontier(...)` evitando duplicati.

Esempio:

- robot in `(0,0)`
- frontiera iniziale: `[(0,1,rank=1), (1,0,rank=9)]`

## T4 - Inizio loop principale

Il loop `while rclpy.ok()` esegue sempre questa logica:

1. se `frontier` vuota -> fine missione.
2. ricalcola cella attuale robot.
3. trova celle di frontiera **adiacenti ora**.
4. se esistono adiacenti -> sceglie quella con rank minimo.
5. altrimenti -> fallback: prende la migliore globale da frontier.

---

## 4) Cosa succede dentro un tentativo di ingresso cella

Supponiamo scelga target `(0,1)`.

### Step A - Preparazione locale

`attempt_enter_cell(env, target_row, target_col)`:

1. costruisce `grid_helper` da `self.current_map`.
2. legge posa robot corrente `(robot_x, robot_y)`.
3. logga allineamento (`ALIGN_PRE`).

### Step B - Campionamento punti cella

`find_best_point_in_cell(...)`:

1. campiona ~100 punti casuali interni alla cella target.
2. per ciascun punto esegue `check_line_of_sight(...)`.
3. divide in:
   - `valid_samples`
   - `rejected_samples`
4. sceglie il valid piu vicino al centro cella.

Se `valid_samples` e' vuoto:

- esito `LOS_BLOCKED`
- la cella viene gestita come tentativo fallito.

### Step C - Filtro locale anti-urto

Prima del movimento il target viene filtrato con osservazioni locali:

- `self._is_local_obstacle_near(target_x, target_y)` controlla:
  - nuvola locale da scan recente,
  - opzionale camera local grid recente.

Se target locale occupato:

1. prova un altro punto tra i `valid_samples` che sia localmente safe,
2. se non esiste -> `LOCAL_OBS_BLOCKED`.

### Step D - Visualizzazione RViz

`visualize_grid_with_candidates_ros(...)` pubblica MarkerArray con:

- occupancy locale (occupied/padding/free/unknown),
- celle visitate/bloccate,
- campioni valid/rejected,
- target scelto,
- distanza robot-target,
- waypoint e path.

### Step E - Movimento sicuro a step

`_safe_move_to_target(target_x, target_y, grid_helper)`:

1. avanza a piccoli step (`avoid_step_size`), non in un unico salto.
2. ogni step verifica corridoio con `_is_corridor_clear(...)`.
3. `_is_segment_clear(...)` blocca se:
   - mappa globale oltre soglia (`avoid_occupied_threshold` o `avoid_partial_threshold`),
   - ostacolo vicino da osservazione locale.

Se il corridoio e' libero:

- invia movimento (`_move_to_world_target` -> `motion.move_to`).

Se bloccato:

- tenta sidestep sinistra/destra (`avoid_lateral_offset`),
- riprova fino a `avoid_max_retries`.

Errori tipici possibili:

- `AVOIDANCE_BLOCKED`
- `AVOIDANCE_STALLED`
- `AVOIDANCE_MAX_STEPS`
- `TF_UNAVAILABLE`
- `MOVE_NO_PROGRESS`

### Step F - Verifica finale cella

Dopo movimento:

1. legge posa finale,
2. `env.is_point_in_cell(...)` verifica appartenenza alla cella target.

Se dentro -> `OK`.
Se fuori -> `CELL_MISMATCH`.

---

## 5) Cosa succede dopo ogni tentativo

## Caso successo

Se `attempt_enter_cell(...)` ritorna `OK`:

1. `env.mark_cell_visited(row,col)`.
2. crea waypoint (stub recording in ROS mode).
3. aggiunge waypoint/posizione a `env`.
4. aggiorna frontier con nuove celle adiacenti.
5. rimuove la cella appena tentata dalla frontier.

Effetto: il robot espande la regione esplorata in modo locale.

## Caso fallimento

Se il tentativo fallisce:

1. cella rimossa dalla frontier corrente,
2. `_handle_failed_attempt(...)` decide:
   - errore transiente -> reinserisce la cella per retry (entro limite),
   - errore permanente -> `env.mark_cell_blocked(row,col)`.
3. per certi errori prova retreat verso waypoint precedente.

Effetto: evita di bloccarsi in loop ciechi sulla stessa cella.

---

## 6) Esempio narrativo breve di una run

Immagina questa sequenza reale:

1. Entra in `(0,1)` con successo.
2. Aggiorna frontier con `(0,2)`, `(1,1)`.
3. Prova `(0,2)`: LOS valida, ma scan locale vede ostacolo improvviso -> cambia target interno cella.
4. Muove a step, al terzo step corridoio bloccato -> sidestep LEFT riuscito.
5. Completa ingresso cella -> mark visited.
6. Prova `(1,2)`: tutti i campioni valid risultano localmente occupati -> `LOCAL_OBS_BLOCKED`.
7. Retry una volta (errore transiente), secondo tentativo ancora fallito -> cella marcata bloccata.
8. Continua su altra frontiera disponibile a rank min.
9. Quando frontier diventa vuota -> rientra al punto iniziale con `_return_to_start_via_robot_path`.

---

## 7) Perche' ora e' piu fedele alla logica local-grid

Rispetto a una strategia solo map-based:

- non si fida solo della `/map` globale,
- usa un layer locale recente (scan e opzionale camera local grid),
- blocca anche celle parzialmente occupate (`avoid_partial_threshold`),
- ricampiona target nella stessa cella se il primo punto e' localmente a rischio,
- usa sidestep reattivo quando il corridoio si chiude.

Questa e' la parte che replica meglio il comportamento "local perception first" del flusso SDK.

---

## 8) Parametri che influenzano di piu il comportamento

Se vuoi fare tuning, questi sono i piu impattanti:

- `avoid_occupied_threshold` / `avoid_partial_threshold`
- `avoid_step_size`
- `avoid_corridor_half_width`
- `avoid_lateral_offset`
- `avoid_max_retries`
- `local_obs_timeout`
- `local_obs_hit_radius`
- `camera_local_grid_occupied_threshold`

Regola pratica:

- soglie piu basse + raggio hit piu alto => comportamento piu prudente,
- step grandi + soglie alte => comportamento piu aggressivo (rischio urti maggiore).

---

## 9) Stato finale missione

La missione e' considerata conclusa quando:

1. non ci sono piu celle in `frontier`,
2. il robot esegue ritorno su path registrato,
3. viene loggato `Exploration complete.`

A quel punto la run e' completa: celle visitate/bloccate aggiornate, traccia percorso e visualizzazione RViz disponibili.
