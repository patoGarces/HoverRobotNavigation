# HoverRobotNavigation

Sistema completo de navegación autónoma para HoverRobot ESP32 basado en ROS2, con capacidades de SLAM, control remoto, camara PS5, procesamiento de imágenes y comunicación serial.

## Indice

- [Descripción General](#descripción-general)
- [Arquitectura del Proyecto](#arquitectura-del-proyecto)
- [Packages del Proyecto](#packages-del-proyecto)
- [Pipeline de Procesamiento de Imágenes](#pipeline-de-procesamiento-de-imágenes-y-profundidad)
- [Nodos del Sistema](#nodos-del-sistema)
- [Instalación](#instalación)
- [Uso](#uso)
- [Dependencias](#dependencias)
- [Notas adicionales](#notas-adicionales)
- [Contribuciones](#contribuciones)
- [Licencia](#licencia)
- [Autor](#autor)

---

## Descripción General

HoverRobotNavigation es un proyecto ROS2 que implementa navegación autónoma utilizando la cámara estéreo del control PS5 para generar mapas de profundidad. El sistema convierte la información de disparidad en datos de tipo LaserScan, permitiendo el uso de algoritmos de SLAM 2D estándar para mapeo y localización.

### Características Principales

- **Visión Estéreo PS5**: Utiliza las cámaras de PlayStation 5 para obtener un par de frames stereo
- **SLAM 2D**: Mapeo y localización utilizando SLAM Toolbox y Cartographer
- **Comunicación Serial**: Interfaz con el hardware del robot
- **Descripción del Robot**: Publicación de URDF y transformaciones TF

---

## Arquitectura del Proyecto

```
HoverRobotNavigation/
├── src/
│   ├── ros2_ps5_stereo/           # Captura y procesamiento de cámaras PS5
│   │
│   ├── disparity_to_laserscan_cpp/  # Conversión disparidad → laserscan
│   │
│   ├── hoverrobot_navigation/      # Configuración de SLAM y navegación
│   │
│   ├── ros2_hoverrobot_comms/      # Comunicación con hardware del robot
│   │
│   └── hoverrobot_description/     # Descripción URDF del robot
│
├── rvizConfig.rviz
└── frames_*.pdf                    # Diagramas del árbol TF
```

### Flujo de Datos Principal

```
Cámaras PS5 → Imágenes Estéreo → Mapa de Disparidad → LaserScan Virtual → SLAM → Mapa 2D
                                                              ↑
                                                         Odometría ← Comms ← Hardware
```

---

## Packages del Proyecto

### 1. ros2_ps5_stereo

**Descripción**: Maneja la captura y procesamiento de las cámaras estéreo PS5.

**Funcionalidades**:
- Inyección de firmware en runtime
- Captura de frames de ambas cámaras del PS5
- Calibración estéreo
- Cálculo de mapa de disparidad
- Publicación de imágenes y datos de disparidad

**Módulos principales**:
- `camera_node.py`: Nodo principal de ROS2 para captura de cámaras
- `cameraPs5Handler.py`: Handler para interactuar con las cámaras PS5
- `controlCamera.py`: Control y configuración de parámetros de cámara
- `getFrame.py`: Captura de frames individuales
- `Utils/utils.py`: Funciones auxiliares para procesamiento

**Topics publicados**:
- `/left/image_raw` (sensor_msgs/Image): Imagen de cámara izquierda
- `/right/image_raw` (sensor_msgs/Image): Imagen de cámara derecha
- `/left/camera_info` (sensor_msgs/CameraInfo): Parametros de cámara izquierda
- `/right/camera_info` (sensor_msgs/CameraInfo): Parametros de cámara derecha
- `/ps5/disparity` (stereo_msgs/DisparityImage): Mapa de disparidad

**Topics suscritos**:
- Ninguno (fuente de datos del sistema)

**Tipo de nodos**: Custom (Python)

---

### 2. disparity_to_laserscan_cpp

**Descripción**: Convierte el mapa de disparidad generado por las cámaras estéreo en un mensaje de tipo LaserScan, permitiendo usar algoritmos de SLAM 2D.

**Funcionalidades**:
- Conversión de disparidad a distancia
- Proyección de puntos 3D a un plano 2D
- Generación de scan láser virtual en 2D
- Filtrado de puntos válidos
- Configuración de rango y resolución angular

**Archivo principal**:
- `disparity_to_laserscan.cpp`: Implementación en C++ del algoritmo de conversión

**Topics suscritos**:
- `/ps5/disparity` (stereo_msgs/DisparityImage): Mapa de disparidad de entrada

**Topics publicados**:
- `/scan` (sensor_msgs/LaserScan): LaserScan virtual generado

**Tipo de nodo**: Custom (C++)

**Parámetros configurables**:
- `scan_height`: Altura en la imagen de disparidad a procesar
- `range_min`: Rango mínimo del scan (metros)
- `range_max`: Rango máximo del scan (metros)
- `angle_min`: Ángulo mínimo del scan (radianes)
- `angle_max`: Ángulo máximo del scan (radianes)
- `scan_time`: Tiempo entre scans (segundos)

---

### 3. hoverrobot_navigation

**Descripción**: Contiene toda la configuración y lanzadores para SLAM, navegación y gestión del stack de navegación.

**Funcionalidades**:
- Configuración de SLAM Toolbox
- Configuración de Cartographer
- Lanzadores integrados del sistema
- Parámetros de navegación


**Configuraciones**:
- `cartographer_config.lua`: Configuración de Cartographer SLAM
- `mapper_params_online_async.yaml`: Parámetros para SLAM Toolbox (modo async)
- `slam_toolbox.yaml`: Configuración general de SLAM Toolbox
- `params.yaml`: Parámetros generales del sistema

**Launch files**:
- `bringup.launch.py`: Lanzador principal del sistema completo
- `slam.launch.py`: Lanzador de SLAM Toolbox
- `cartographer.launch.py`: Lanzador de Cartographer
- `nav2_local.launch.py`: Navegación local con Nav2

**Tipo de nodos**: Estándar ROS2

---

### 4. ros2_hoverrobot_comms

**Descripción**: Gestiona la comunicación bidireccional con el hardware del robot mediante interfaz serial.

**Funcionalidades** (inferidas):
- Envío de comandos de velocidad al microcontrolador
- Recepción de datos de odometría
- Publicación de datos de encoders
- Publicación de datos de IMU (si disponible)
- Manejo de protocolo de comunicación serial

**Topics suscritos** (típicos):
- `/cmd_vel` (geometry_msgs/Twist): Comandos de velocidad

**Topics publicados** (típicos):
- `/odom` (nav_msgs/Odometry): Odometría del robot
- `/joint_states` (sensor_msgs/JointState): Estado de las ruedas

**Tipo de nodos**: Custom

---

### 5. hoverrobot_description

**Descripción**: Contiene la descripción URDF/XACRO del robot, incluyendo geometría, cinemática y propiedades visuales.

**Funcionalidades**:
- Definición del modelo del robot
- Configuración del árbol de transformaciones (TF)
- Meshes y modelos 3D
- Parámetros físicos del robot

**Contenido**:
- Archivos URDF/XACRO
- Meshes (.stl, .dae)
- Launch files para publicar el estado del robot

**Nodos utilizados**: 
- `robot_state_publisher` (ROS2 estándar): Publica transformaciones TF

---

## Pipeline de Procesamiento de Imágenes y Profundidad

El sistema utiliza las cámaras estéreo del control PS5 para generar un mapa de profundidad que luego se convierte en datos láser para SLAM. Este es el pipeline completo:

### 1. Captura de Imágenes Estéreo

**Package**: `ros2_ps5_stereo`  
**Nodo**: `camera_node` (custom)

**Proceso**:
1. Inicialización de las cámaras del control PS5
2. Captura sincronizada de frames izquierdo y derecho
3. Calibración estéreo (parámetros intrínsecos y extrínsecos)
4. Publicación de imágenes raw

**Output**:
- `/left/image_raw`: Imagen de cámara izquierda (640x480 típico)
- `/right/image_raw`: Imagen de cámara derecha (640x480 típico)
- `/left/camera_info`: Información de calibración
- `/right/camera_info`: Información de calibración

**Frecuencia**: ~30 Hz

---

### 2. Cálculo de Disparidad

**Package**: `ros2_ps5_stereo`  
**Módulos**: `cameraPs5Handler.py`, `controlCamera.py`

```
Imagen Izquierda + Imagen Derecha → Algoritmo Estéreo → Mapa de Disparidad
```

**Proceso**:
1. **Rectificación**: Las imágenes se rectifican usando parámetros de calibración
2. **Correspondencia estéreo**: Se buscan píxeles correspondientes entre ambas imágenes
3. **Cálculo de disparidad**: Se calcula la diferencia de posición horizontal
4. **Post-procesamiento**: Filtrado y validación de disparidades

**Algoritmos posibles**:
- Block Matching (BM)
- Semi-Global Block Matching (SGBM)
- Algoritmos acelerados por GPU

**Output**:
- `/ps5/disparity` (stereo_msgs/DisparityImage): Mapa de disparidad

---

### 3. Generación de Mapa de Profundidad

**Package**: `ros2_ps5_stereo`

```
Mapa de Disparidad + Parámetros de Calibración → Mapa de Profundidad
```

**Fórmula**:
```
Profundidad (Z) = (f × B) / disparidad

donde:
- f = distancia focal de la cámara
- B = baseline (separación entre cámaras)
- disparidad = diferencia de píxeles entre imágenes
```

**Proceso**:
1. Conversión disparidad → profundidad usando parámetros de calibración
2. Generación de nube de puntos 3D (opcional)
3. Proyección en imagen de profundidad

**Output**:
- `/ps5/depth` (sensor_msgs/Image): Imagen de profundidad
- `/ps5/points` (sensor_msgs/PointCloud2): Nube de puntos 3D (opcional)

---

### 4. Conversión a LaserScan Virtual

**Package**: `disparity_to_laserscan_cpp`  
**Nodo**: `disparity_to_laserscan` (custom, C++)

```
Mapa de Disparidad → Proyección 2D → LaserScan Virtual
```

**Proceso**:
1. **Selección de altura**: Se toma una línea horizontal del mapa de disparidad
2. **Conversión a distancia**: Se convierte disparidad a distancia para cada píxel
3. **Proyección angular**: Se mapean píxeles a ángulos del scan
4. **Filtrado**: Se eliminan puntos fuera de rango o inválidos
5. **Generación de LaserScan**: Se crea el mensaje con formato estándar

**Parámetros clave**:
- `scan_height`: Fila de la imagen a procesar (altura del "láser virtual")
- `range_min/max`: Rango de detección válido (ej: 0.1m - 10m)
- `angle_min/max`: Campo de visión angular (ej: -45° a +45°)

**Output**:
- `/scan` (sensor_msgs/LaserScan): Scan láser virtual en 2D

**Ventajas de este enfoque**:
- Permite usar algoritmos de SLAM 2D sin modificación
- Compatible con toda la toolchain de navegación ROS2
- Menor costo computacional que SLAM 3D
- Aprovecha robustez de algoritmos láser 2D

---

### 6. SLAM y Mapeo

**Package**: `hoverrobot_navigation`  
**Nodos**: SLAM Toolbox o Cartographer (ROS2 estándar)

```
/scan + /odom → SLAM → Mapa 2D + Localización
```

**Opciones de SLAM**:

#### A) SLAM Toolbox
- Configuración: `mapper_params_online_async.yaml`
- Modo asíncrono para mejor rendimiento
- Soporte para cierre de bucle
- Serialización de mapas

#### B) Cartographer
- Configuración: `cartographer_config.lua`
- SLAM basado en graph optimization
- Buen rendimiento con scans ruidosos

**Output**:
- `/map` (nav_msgs/OccupancyGrid): Mapa de ocupación 2D
- `/map_metadata`: Metadatos del mapa
- Transformación `map` → `odom`

---

### Diagrama Completo del Pipeline

```
┌─────────────────────┐
│  Control PS5        │
│  (Cámaras Estéreo)  │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  camera_node        │
│  - Captura frames   │
│  - Sincronización   │
└──────────┬──────────┘
           │
           ├─► /ps5/left/image_raw
           ├─► /ps5/right/image_raw
           │
           ▼
┌─────────────────────┐
│  Procesamiento      │
│  Estéreo            │
│  - Rectificación    │
│  - Block Matching   │
└──────────┬──────────┘
           │
           ├─► /ps5/disparity
           ├─► /ps5/depth
           │
           ▼
┌─────────────────────┐
│  disparity_to_      │
│  laserscan          │
│  - Proyección 2D    │
│  - Generación scan  │
└──────────┬──────────┘
           │
           ├─► /scan
           │
           ▼
┌─────────────────────┐
│  SLAM               │
│  (Toolbox/Carto)    │
│  - Mapeo            │
│  - Localización     │
└──────────┬──────────┘
           │
           ├─► /map
           └─► TF: map→odom

┌─────────────────────┐
│  Odometría          │
│  (ros2_hoverrobot_  │
│   comms)            │
└──────────┬──────────┘
           │
           └─► /odom
```

---

## Nodos del Sistema

### Resumen de Nodos

| Nodo | Package | Tipo | Entrada | Salida | Descripción |
|------|---------|------|---------|--------|-------------|
| `camera_node` | ros2_ps5_stereo | Custom (Python) | Hardware PS5 | `/ps5/left/image_raw`, `/ps5/right/image_raw`, `/ps5/disparity`, `/ps5/depth` | Captura de cámaras estéreo y cálculo de disparidad |
| `disparity_to_laserscan` | disparity_to_laserscan_cpp | Custom (C++) | `/ps5/disparity` | `/scan` | Convierte disparidad a LaserScan virtual |
| `async_slam_toolbox_node` | hoverrobot_navigation | Estándar | `/scan`, `/odom` | `/map`, TF (map→odom) | SLAM asíncrono |
| `cartographer_node` | hoverrobot_navigation | Estándar | `/scan`, `/odom` | `/map`, TF (map→odom) | SLAM con Cartographer |
| `comms_node` | ros2_hoverrobot_comms | Custom | `/cmd_vel` | `/odom`, `/joint_states` | Comunicación con hardware |
| `robot_state_publisher` | hoverrobot_description | Estándar | URDF, `/joint_states` | TF transforms | Publica transformaciones del robot |

---

## Instalación

### Requisitos Previos

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+
- GCC 9+

### Instalación de Dependencias

```bash
# Dependencias de ROS2
sudo apt update
sudo apt install -y \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-vision-msgs

# Dependencias de Python
pip3 install \
    pyserial \
    opencv-python \
    numpy \
    scipy
```

### Compilación del Proyecto

```bash
# Clonar el repositorio
cd ~/ros2_ws/src
git clone https://github.com/patoGarces/HoverRobotNavigation.git

# Compilar
cd ~/ros2_ws
colcon build --symlink-install

# Source del workspace
source install/setup.bash
```

---

## Uso

### Lanzar el Sistema Completo

```bash
# Terminal 1: Descripción del robot y estado
ros2 launch hover_description robot_state.launch.py

# Terminal 2: Comunicación con hardware
ros2 launch hover_comms serial_comm.launch.py

# Terminal 3: SLAM
ros2 launch hover_slam slam.launch.py

# Terminal 4: Control PS5
ros2 launch hover_ps5 ps5_teleop.launch.py

# Terminal 5: Visión (opcional)
ros2 launch hover_vision vision_pipeline.launch.py

# Terminal 6: RViz
rviz2 -d rvizConfig.rviz
```

### Lanzadores Integrados

```bash
# Todo en uno
ros2 launch hover_bringup full_system.launch.py

# Solo navegación autónoma
ros2 launch hover_bringup autonomous_nav.launch.py

# Solo teleoperation
ros2 launch hover_bringup teleop.launch.py
```

---

## Dependencias

### Dependencias ROS2

- `slam_toolbox`: SLAM 2D
- `navigation2`: Stack de navegación
- `joy`: Driver de joystick
- `teleop_twist_joy`: Teleoperation
- `robot_state_publisher`: Estado del robot
- `cv_bridge`: Puente OpenCV-ROS
- `vision_msgs`: Mensajes de visión

### Dependencias de Sistema

- `opencv`: Procesamiento de imágenes
- `pyserial`: Comunicación serial
- `numpy`: Operaciones numéricas

---

## Notas Adicionales

### Árbol de Transformaciones (TF)

El robot utiliza el siguiente árbol de frames:

```
map
 └── odom
      └── base_link
           ├── base_footprint
           ├── laser_frame
           ├── camera_link
           │    └── camera_optical_frame
           ├── imu_link
           ├── left_wheel
           └── right_wheel
```

### Topics Principales

| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Comandos de velocidad |
| `/odom` | nav_msgs/Odometry | Odometría |
| `/scan` | sensor_msgs/LaserScan | Datos LIDAR |
| `/map` | nav_msgs/OccupancyGrid | Mapa de ocupación |
| `/camera/image_raw` | sensor_msgs/Image | Imagen de cámara |
| `/detections` | vision_msgs/Detection2DArray | Objetos detectados |
| `/joy` | sensor_msgs/Joy | Estado del joystick |

---

## Contribuciones

Las contribuciones son bienvenidas. Por favor:

1. Fork el proyecto
2. Crea una rama para tu feature (`git checkout -b feature/AmazingFeature`)
3. Commit tus cambios (`git commit -m 'Add some AmazingFeature'`)
4. Push a la rama (`git push origin feature/AmazingFeature`)
5. Abre un Pull Request

---

## Licencia

Este proyecto está bajo licencia MIT. Ver archivo `LICENSE` para más detalles.

---

## Autor

**Patricio Garces**
- GitHub: [@patoGarces](https://github.com/patoGarces)

---

## Enlaces Útiles

- [Documentación ROS2 Humble](https://docs.ros.org/en/humble/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 Documentation](https://navigation.ros.org/)
