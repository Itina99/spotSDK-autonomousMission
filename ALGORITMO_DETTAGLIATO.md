# Spiegazione dettagliata dell'algoritmo di esplorazione autonoma

## 1) Obiettivo del progetto

Questo progetto implementa una missione di esplorazione autonoma per Spot basata su:

- una **griglia globale** (celle visitate/bloccate/sconosciute),
- una **mappa locale ostacoli** derivata dal Local Grid service,
- una **strategia frontier + ranking serpentino** per scegliere la prossima cella,
- integrazione con **GraphNav** per creare waypoint e gestire navigazione di ritorno/riconnessione.

L'orchestrazione principale e' in `easy_walk.py` (funzione `easy_walk`).

---

## 2) Architettura logica (componenti principali)

- `easy_walk.py`
  - ciclo principale di esplorazione;
  - selezione target;
  - campionamento punti nella cella;
  - comandi di movimento e aggiornamento stato missione.

- `environmentMap.py` (`EnvironmentMap`)
  - rappresentazione griglia globale;
  - conversioni world <-> cell;
  - ranking serpentino e gestione frontiera;
  - stato celle (`0` sconosciuta, `1` visitata, `-1` bloccata).

- `spotGrid.py`
  - conversione dati Local Grid (`obstacle_distance`) in punti + valori + colori;
  - trasformazione nel frame `VISION`.

- `navGraphUtils.py` (`RecordingInterface`)
  - controllo recording GraphNav;
  - creazione waypoint manuali `wp_N`;
  - download grafo e snapshot;
  - navigazione a waypoint con gestione recovery (es. `STATUS_LOST`).

- `movements.py`
  - primitive di movimento relativo (`relative_move`) su traiettoria SE2.

- `spotUtils.py`
  - lettura posa robot in `VISION` (`getPosition`).

- `spotLogInUtils.py`
  - login SDK, lease, robot state client, metadata recording, E-Stop.

---

## 3) Modello dati e convenzioni

### 3.1 Stato mappa globale

In `EnvironmentMap.map`:

- `0` = cella non ancora testata,
- `1` = cella visitata/accessibile,
- `-1` = cella tentata ma bloccata.

### 3.2 Origine e allineamento griglia

L'origine viene fissata all'avvio missione (`env.set_origin`) usando la posa boot del robot:

- `origin_x`, `origin_y` = posizione iniziale,
- `origin_yaw` = orientamento iniziale,
- `start_cell` = cella iniziale (tipicamente `(0,0)`).

Le conversioni coordinate tengono conto di `origin_yaw`:

- world -> grid: rotazione inversa,
- grid -> world: rotazione diretta.

### 3.3 Frame usati

- `VISION`: frame principale per posizione robot, celle e waypoint salvati.
- `BODY`: frame corpo robot (usato nelle trasformazioni).
- `ODOM`: usato in GraphNav per localizzazione e trasformazioni interne.

La coerenza su `VISION` e' centrale per confronti distanza (robot-target, waypoint-target).

---

## 4) Flusso operativo completo della missione

## 4.1 Inizializzazione

In `easy_walk.easy_walk(options)`:

1. Login robot + lease + robot state (`spotLogInUtils.setLogInfo`).
2. Setup E-Stop (`SimpleEstop`).
3. Creazione `RecordingInterface` GraphNav.
4. `stop_recording()` e `clear_map()` iniziali.
5. Power on e stand (`blocking_stand`).
6. `start_recording()`.
7. Tentativo opzionale di localizzazione via fiducial (`initialize_with_fiducial`).
8. Creazione primo waypoint `wp_0`.
9. Inizializzazione `EnvironmentMap` e origine.
10. Creazione cartelle missione (`MissionMap`, `graph`, `MissionLogs`).
11. Generazione percorso serpentino (`generate_serpentine_path`).

## 4.2 Costruzione e gestione frontiera

La frontiera e' una lista di celle adiacenti esplorabili, arricchite con rank nel percorso serpentino:

- `find_new_borders(...)` usa `env.get_adjacent_frontier_cells(...)`;
- vengono aggiunte solo celle non visitate e non gia' in frontiera.

Scelta target:

- se ci sono celle adiacenti in frontiera -> si prende quella con **rank minimo**;
- altrimenti -> si seleziona la migliore dalla frontiera globale (`get_lowest_rank_from_frontier_list`).

## 4.3 Tentativo di ingresso in cella (`attempt_enter_cell_from_position`)

Per entrare in una cella target:

1. legge la local grid `obstacle_distance`;
2. converte la griglia in punti world (`create_vtk_obstacle_grid`);
3. recupera posizione robot corrente;
4. campiona punti candidati nella cella (`sample_cell_points`);
5. valuta linea di vista robot->punto (`check_line_of_sight`);
6. sceglie il candidato valido piu' vicino al centro cella (`find_best_point_in_cell`);
7. ruota verso target e poi avanza (`movements.relative_move`);
8. verifica finale: robot dentro la cella (`env.is_point_in_cell`).

Se nessun punto e' raggiungibile, il tentativo fallisce.

## 4.4 Aggiornamento stato dopo successo

Quando l'ingresso riesce:

- viene creato un nuovo waypoint manuale (`create_default_waypoint`),
- viene aggiunta la posa waypoint in `env.waypoints`,
- la cella viene marcata visitata (`mark_cell_visited`),
- la frontiera viene aggiornata con nuovi adiacenti.

## 4.5 Caso non adiacente: navigazione via waypoint

Quando non ci sono celle frontiera adiacenti:

1. seleziona target a rank minimo dalla frontiera;
2. ferma recording;
3. ricava waypoints manuali con metadati cella;
4. trova waypoint "piu' vicino" alla cella target (`find_nearest_waypoint_cell_to_target`);
5. naviga al waypoint (`navigate_to_waypoint`);
6. riavvia recording;
7. tenta ingresso cella con la stessa procedura locale.

## 4.6 Chiusura missione

A frontiera vuota:

- crea waypoint finale,
- richiama loop closure (`auto_close_loops`),
- stop recording,
- ottimizza anchoring (`optimize_anchoring`),
- ritorna a `wp_0` (`navigate_to_first_waypoint`),
- sit + power off,
- download completo grafo (`download_full_graph`).

---

## 5) Metodologie usate (focus richiesto)

### 5.1 Frontier-based exploration con ranking serpentino

La metodologia combina due idee:

1. **Localita'**: priorita' alle celle adiacenti all'attuale posizione (espansione locale).
2. **Ordine globale**: ranking della cella nel percorso serpentino per risolvere conflitti e ripartenze.

Vantaggi:

- evita salti non necessari quando esiste continuita' locale;
- mantiene una progressione globale deterministica;
- semplifica la copertura sistematica della mappa.

### 5.2 Campionamento stocastico intra-cella + test LOS

Invece di puntare direttamente al centro cella, l'algoritmo:

- genera molti campioni random interni;
- esegue test di line-of-sight su ciascun campione;
- sceglie il migliore tra i validi (min distanza dal centro).

Questo riduce fallimenti in presenza di ostacoli parziali dentro la cella o vicino ai bordi.

### 5.3 Validazione geometrica con signed distance map

`obstacle_distance` fornisce una distanza signed dall'ostacolo:

- `< 0`: dentro ostacolo,
- `>= 0`: area attraversabile.

Nel controllo LOS viene usata una soglia (`obstacle_threshold`) per avere un margine conservativo durante la scelta dei punti.

### 5.4 Strategia ibrida locale + topologica

La navigazione non e' solo reattiva locale:

- locale: entrata in cella tramite local grid;
- topologica: spostamenti lunghi tramite waypoint GraphNav.

Questa combinazione aumenta robustezza in scenari ampi o con corridoi/ostacoli complessi.

### 5.5 Recovery operativo

Sono presenti meccanismi di recupero:

- in `navigate_to_waypoint`, se `STATUS_LOST`, viene tentata `force_localization_to_waypoint`;
- fallback e logging nei casi di `STUCK`/errori navigazione;
- ottimizzazione anchoring finale per consistenza metrica del grafo.

---

## 6) Sezione dettagliata Spot SDK (focus richiesto)

## 6.1 Client e servizi usati

### Accesso robot

In `spotLogInUtils.setLogInfo`:

- creazione SDK (`create_standard_sdk`),
- creazione robot handle (`sdk.create_robot`),
- autenticazione,
- time sync,
- lease client,
- robot state client,
- metadata client recording.

### Controllo sicurezza

`SimpleEstop` usa:

- `EstopClient`, `EstopEndpoint`, `EstopKeepAlive`.

### Motion command

In `movements.relative_move`:

- `RobotCommandBuilder.synchro_se2_trajectory_point_command`,
- feedback continuo da `robot_command_feedback`.

### Local Grid

In `easy_walk` + `spotGrid`:

- `LocalGridClient.get_local_grids(['obstacle_distance'])`,
- parsing/decodifica griglia (raw o RLE),
- trasformazione nel frame `VISION`.

### GraphNav / Recording

In `navGraphUtils.RecordingInterface`:

- `GraphNavClient`,
- `GraphNavRecordingServiceClient`,
- `MapProcessingServiceClient`.

Operazioni principali:

- `start_recording`, `stop_recording`, `clear_map`;
- `create_waypoint` (manual waypoint `wp_N`);
- `navigate_to` e feedback;
- `set_localization` (fiducial o waypoint);
- `process_topology` (loop closure);
- `process_anchoring` (ottimizzazione globale);
- download graph/waypoint snapshots/edge snapshots.

## 6.2 Localizzazione e frame nel dettaglio

Il codice usa in modo esplicito:

- `get_a_tform_b(snapshot, VISION, BODY)` per posa robot in world operativo;
- `get_odom_tform_body(...)` come input in alcuni step GraphNav (`set_localization`).

Impatto pratico:

- decisioni di esplorazione e geometria cella sono in `VISION`;
- GraphNav mantiene internamente la coerenza topologica/odometrica e viene aggiornato via record + anchoring.

## 6.3 Registrazione waypoints manuali con metadati cella

Ogni waypoint creato (`create_default_waypoint`) salva:

- nome (`wp_N`),
- ID waypoint GraphNav,
- posa (`x,y,z,yaw`),
- indice cella (`cell_row`, `cell_col`) se noto.

Questo abilita il bridging tra:

- livello continuo metrico (pose reali) e
- livello discreto griglia (celle).

## 6.4 Navigazione GraphNav con gestione errori

`navigate_to_waypoint` implementa un loop comando/feedback:

- successo: `STATUS_REACHED_GOAL`;
- perdita localizzazione: `STATUS_LOST` -> recovery forzando localizzazione sul waypoint target;
- blocco: `STATUS_STUCK` -> fail esplicito.

`navigate_to_first_waypoint` applica logica analoga per il rientro a base.

## 6.5 Download e persistenza mappa

`download_full_graph` salva su cartella timestamp:

- file `graph`,
- `waypoint_snapshots/`,
- `edge_snapshots/`.

Questo permette post-analisi offline e riuso delle mappe prodotte.

---

## 7) Visualizzazione e tracciamento missione

In `easy_walk.visualize_grid_with_candidates`:

- overlay local grid + griglia globale,
- campioni validi/scartati,
- target scelto,
- waypoint registrati,
- traccia percorso robot (`explore` vs `navigate`),
- salvataggio immagini per iterazione.

E' presente anche una vista globale accumulata delle scansioni locali nel tempo.

---

## 8) Pseudocodice sintetico del ciclo

```text
init robot/services
start recording
(optional) fiducial localization
create wp_0
init EnvironmentMap + origin + serpentine path
frontier <- neighbors(start)

while frontier not empty:
    if esistono frontier adiacenti alla cella robot:
        target <- adiacente con rank min
        success <- attempt_enter_cell(target)
    else:
        target <- frontier globale con rank min
        stop recording
        navigate to nearest waypoint near target
        start recording
        success <- attempt_enter_cell(target)

    if success:
        create waypoint per target
        mark target visited
        aggiorna frontier con nuovi adiacenti
    else:
        rimuovi target da frontier

close loops + optimize anchoring
navigate_to wp_0
sit + power off
download graph
```

---

## 9) Punti di attenzione tecnici

- In `movements.relative_move` il valore di ritorno e' una tupla `(success, distance_traveled)`; in alcuni punti di `easy_walk.py` e' usato come se fosse un booleano puro. Conviene gestire esplicitamente il primo elemento della tupla.
- La scelta del target waypoint per celle lontane e' Manhattan-based sulla griglia; in ambienti molto irregolari puo' non coincidere con la geodetica reale nel grafo.
- Le soglie di classificazione ostacoli (`obstacle_threshold`) influenzano direttamente aggressivita'/sicurezza del planner locale.

---

## 10) In sintesi

L'algoritmo implementa una pipeline robusta e pragmatica:

1. copertura sistematica a celle (serpentina + frontier),
2. validazione locale geometrica con local grid,
3. supporto topologico GraphNav per riallineamenti/spostamenti lunghi,
4. chiusura e consolidamento mappa con loop closure + anchoring.

La parte SDK di Spot e' integrata in modo esteso: controllo robot, sensing locale, registrazione mappe, localizzazione, navigazione e persistenza finale del grafo.

