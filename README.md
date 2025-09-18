
## 📚 Useful resources

- **ROS 2 Cheat Sheet (Jazzy)**  
  [Download PDF](https://s3.amazonaws.com/assets.clearpathrobotics.com/wp-content/uploads/2025/02/06151220/ROS-2-Cheat-Sheet_Jazzy_FINAL.pdf)

- **Uso di Git/Github**  
  [Download PDF](https://people.cs.dm.unipi.it/limco/2021-22/slides/04-GitHub.pdf)

- **Git Cheat Sheet (GitHub)**  
  [Download PDF](https://education.github.com/git-cheat-sheet-education.pdf)

- **Git Cheat Sheet (GitLab)**  
  [Download PDF](https://about.gitlab.com/images/press/git-cheat-sheet.pdf)


# Settings utili

0. Installazione Vscode:
```bash
sudo snap install code --classic
```

1. Configurazione git 

Impostazioni Git: Se avete repository privati dovrete creare un token e usarlo come password quando farete git clone. Comandi che potrebbero servire:

```bash
git config --global credential.helper store # Salvare le credenziali che verranno inserite successivamente
git config --global user.name "name"
git config --global user.email "email"
```

Per vedere differenza tra file è comodo usare Meld. Meld è un’applicazione grafica (GUI) per confrontare file e directory. 

```bash
sudo apt install meld
git config --global diff.tool meld
```

2. Rqt plugin vari: 
```bash
sudo apt install ros-jazzy-rqt*
```

# Lezione 1


# Lezione 2 - Introduzione agli URDF in ROS 2

In questa lezione useremo il pacchetto `urdf_tutorial` come base di esempio per comprendere i file URDF e visualizzarli in ROS 2.

---

## 1. Clonare il repository

Cloniamo il branch ROS 2 del pacchetto:

```bash
cd ~/esn_ws/src
git clone https://github.com/ros/urdf_tutorial.git -b ros2
```

## 2. Compilare il pacchetto

Dalla root del workspace:

```bash
cd ~/esn_ws
colcon build --symlink-install
```

## 3. Source del workspace

Dopo la compilazione, ricordiamoci di fare il source:

```bash
source install/setup.bash
```

## 4. Lanciare i modelli URDF

Il comando generale è:

```bash
ros2 launch urdf_tutorial display.launch.py model:=urdf/<nome_file>.urdf
```

Elenco modelli
```bash
# 01 - Box
ros2 launch urdf_tutorial display.launch.py model:=urdf/01-myfirst.urdf

# 02 - Joint
ros2 launch urdf_tutorial display.launch.py model:=urdf/02-joint.urdf

# 03 - Visual
ros2 launch urdf_tutorial display.launch.py model:=urdf/03-visual.urdf

# 04 - Visual + Collision
ros2 launch urdf_tutorial display.launch.py model:=urdf/04-collision.urdf

# 05 - Materials
ros2 launch urdf_tutorial display.launch.py model:=urdf/05-materials.urdf

# 06 - Macros
ros2 launch urdf_tutorial display.launch.py model:=urdf/06-macro.urdf

# 07 - Physics
ros2 launch urdf_tutorial display.launch.py model:=urdf/07-physics.urdf

# 08 - Robot con macro
ros2 launch urdf_tutorial display.launch.py model:=urdf/08-macro-robot.urdf
```

## 5. Esempio di generazione urdf da file xacro:
```bash
ros2 run xacro xacro 08-macroed.urdf.xacro > test.urdf
# xacro 08-macroed.urdf.xacro > test.urdf
```