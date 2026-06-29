# Twin remoto (leader en laptop, follower en Raspberry)

Arquitectura sin Gazebo: el brazo **líder** (USB, modo pasivo) va en la **laptop**; el **follower** real corre en la **Raspberry**. Solo el enlace de red vive en la laptop; todo lo demás en la Pi.

## Topología

```
[Laptop]  leader (USB)  -->  /leader/joint_states
                |
         remote_joint_states_relay
                |
                v  LAN (ROS 2 DDS)
         /teleop/joint_states
                |
[Raspberry]  mirror  -->  arm_controller / gripper_controller  -->  brazo follower
```

## Requisitos

- Mismo `ROS_DOMAIN_ID` en ambas máquinas (por defecto `42`).
- `rmw_cyclonedds_cpp` instalado en ambas.
- IPs editadas en `config/cyclonedds_laptop.xml` (peer = Pi) y `config/cyclonedds_pi.xml` (peer = laptop).
- Usuario en grupo `dialout` en cada máquina que tenga USB serial.

## Raspberry Pi (follower)

Terminal 1 — entorno de red:

```bash
source ~/lerobot_ws/install/setup.bash
source ~/lerobot_ws/src/lerobot_teleoperation/env/remote_pi.bash
```

Terminal 2 — controlador del brazo follower:

```bash
ros2 launch lerobot_controller controller.launch.py \
  is_sim:=false teleop_follower:=true uart_port:=/dev/ttyACM0
```

Terminal 3 — mirror (lee joint_states por LAN, manda al hardware local):

```bash
ros2 launch lerobot_teleoperation teleop_mirror.launch.py \
  source_joint_states_topic:=/teleop/joint_states \
  target_ns:=/ \
  command_mode:=forward \
  publish_deadband:=0.002 \
  smoothing_alpha:=1.0
```

## Laptop (leader + enlace remoto)

Terminal 1 — entorno de red:

```bash
source ~/Documents/GITHUB/lerobot_ws/install/setup.bash
source src/lerobot_teleoperation/env/remote_laptop.bash
```

Terminal 2 — brazo líder (modo pasivo):

```bash
ros2 launch lerobot_controller controller.launch.py \
  is_sim:=false ns:=leader leader_only:=true uart_port:=/dev/ttyACM0
```

Terminal 3 — relay LAN (única pieza “remota”):

```bash
ros2 launch lerobot_teleoperation teleop_remote_relay.launch.py
```

## Comprobar enlace

En la laptop (con leader activo):

```bash
ros2 topic hz /teleop/joint_states
```

En la Raspberry:

```bash
ros2 topic hz /teleop/joint_states
```

Si la Pi ve el topic y hay Hz, el twin remoto está enlazado. Mueve el líder a mano y el follower debería imitar.

## IPs CycloneDDS

| Archivo | Peer por defecto | Editar a |
|---------|------------------|----------|
| `cyclonedds_laptop.xml` | `10.42.0.70` | IP de tu Raspberry |
| `cyclonedds_pi.xml` | `10.42.0.1` | IP de tu laptop en esa LAN |

## Orden de arranque

1. Raspberry: `remote_pi.bash` → follower controller → mirror  
2. Laptop: `remote_laptop.bash` → leader controller → relay  

## Twin local (dos brazos USB, sin red)

Sigue usando `dual_robot_teleop.launch.py` o los mismos launches sin relay ni CycloneDDS extra.
