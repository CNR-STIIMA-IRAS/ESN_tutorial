# Settings utili

1. Configurazione git 

Per vedere differenza tra file è comodo usare Meld. Meld è un’applicazione grafica (GUI) per confrontare file e directory. 

```bash
sudo apt install meld
git config --global diff.tool meld
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