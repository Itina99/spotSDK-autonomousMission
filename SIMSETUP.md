# Spot Sim: ROS 2 & Gazebo Ignition Setup

Questo repository contiene la configurazione per simulare il robot Boston Dynamics **Spot** utilizzando **ROS 2 Humble** e **Gazebo Ignition Fortress**.

## 1. Requisiti di Sistema
* **OS:** Ubuntu 22.04 LTS
* **ROS 2:** Humble Hawksbill
* **Simulatore:** Gazebo Ignition Fortress

---

## 2. Installazione Dipendenze

Per prima cosa, installa i pacchetti necessari per la visione e l'integrazione tra ROS e OpenCV (assicurandoti di avere Conda disattivato):

```bash
sudo apt update
sudo apt install python3-opencv ros-humble-cv-bridge ros-humble-vision-msgs
```

## 3. Setup del Workspace

Clonazione e compilazione dei driver e dei modelli per Spot:

```bash
# Crea la cartella del workspace
mkdir -p ~/spot_sim_ws/src
cd ~/spot_sim_ws/src

# Clona il repository ufficiale della simulazione
git clone https://github.com/g1y5x3/spot_gazebo_ros2.git

# Torna nella root e compila
cd ~/spot_sim_ws
colcon build --symlink-install
```

## 4. Configurazione Ambiente (Source)

Per far sì che il terminale riconosca ROS 2 e i pacchetti di Spot appena compilati, esegui sempre questi comandi in ogni nuovo terminale:

```bash
source /opt/ros/humble/setup.bash
source ~/spot_sim_ws/install/setup.bash
```

Consiglio: aggiungi queste righe in fondo al tuo file `~/.bashrc` per non doverle scrivere ogni volta che apri un terminale.

## 5. Configurazione del Mondo (SDF)

Il file del mondo originale conteneva troppi oggetti (tunnel, barriere). Abbiamo creato un file pulito chiamato `mondo_pulito.sdf` che contiene solo l'essenziale per far funzionare Spot senza farlo cadere nel vuoto:

* Un Ground Plane (pavimento)
* Una Luce Direzionale (Sole)
* Il modello di Spot inserito direttamente nel mondo (`<model name="spot">`)

Il file si trova al percorso: `~/spot_sim_ws/mondo_pulito.sdf`

## 6. Lancio della Simulazione

Per avviare Gazebo con il mondo pulito e lo spawner di ROS attivo:

```bash
ros2 launch spot_bringup spot.gazebo.launch.py world_file:=~/spot_sim_ws/mondo_pulito.sdf
```

## 7. Comandi di Movimento (Hello World)

### Test rapido da Terminale

Fallo camminare in cerchio per verificare che i controller dei motori stiano ricevendo input:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.4}}"
```

### Esecuzione Script Python (ROS 2)

Abbiamo tradotto lo script "Hello Spot" originale in un nodo ROS 2 puro (`hello_spot_ros.py`).

* Assicurati che Conda sia disattivato (`conda deactivate`)
* Esegui il source degli ambienti (vedi punto 4)
* Lancia lo script:

```bash
python3 hello_spot_ros.py
```

Lo script eseguirà una sequenza preimpostata:

* Rotazione sul posto
* Avanzamento dritto
* Scatto e salvataggio di un'immagine dalla telecamera termica simulata (`hello_spot_ros_image.jpg`)

## Note Importanti e Risoluzione Problemi

* **Conda vs ROS 2:** Non mischiare mai ambienti Conda con script ROS 2. Conda sovrascrive i percorsi delle librerie di sistema (come OpenCV e C++) causando errori di compilazione o Segmentation Fault in `cv_bridge`.

* **PyCharm IDE:** Se usi PyCharm e ricevi l'errore `ModuleNotFoundError: No module named 'rclpy'`, assicurati di aver fatto il source nel terminale di PyCharm e di aver impostato l'interprete su System Interpreter (`/usr/bin/python3`), disattivando l'ambiente virtuale (venv) generato automaticamente dall'IDE.