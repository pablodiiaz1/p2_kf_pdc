# p2_kf_pdc

Implementación de dos variantes de Filtro de Kalman en ROS 2 para estimación de posición y velocidad de un robot móvil.

---

## Descripción del proyecto

En esta práctica se aborda la estimación del estado de un robot móvil utilizando el Filtro de Kalman (KF) en su forma lineal. Se implementan dos modelos de estado:

1. **Modelo simplificado**: estima únicamente la posición 
   - Estado: \([x, y, 	heta]\)
   - Dinámica linealizada con matriz de transición identidad y control basado en velocidad y orientación.

2. **Modelo completo**: estima posición y velocidad 
   - Estado: \([x, y, v_x, v_y, \omega]\)
   - Dinámica lineal con inclusión explícita de velocidades lineales y angulares.

El objetivo es comparar cómo distintas configuraciones de covarianzas de proceso (Q) y medición (R) afectan la calidad de la estimación bajo ruido bajo, ruido alto en la medición y ruido alto en el proceso.

---

## Estructura del repositorio

```
p2_kf_pdc/
├── filters/                   # Implementación de filtros
│   └── kalman_filter.py       # Lógica base del KF
├── src/p2_kf_pdc/             # Código fuente ROS 2
│   ├── kf_estimation.py       # Nodo: modelo simplificado
│   ├── kf_estimation_vel.py   # Nodo: modelo completo
│   ├── motion_models.py       # Definición del modelo de movimiento
│   ├── observation_models.py  # Definición de modelo de observación
│   ├── sensor_utils.py        # Simulación de ruido y adquisición odométrica
│   └── visualization.py       # Visualización y guardado de gráficas
└── graficas/                  # Resultados: trayectorias estimadas
    ├── simplified_low_noise.png
    ├── simplified_high_measurement_noise.png
    ├── simplified_high_process_noise.png
    ├── complete_low_noise.png
    ├── complete_high_measurement_noise.png
    └── complete_high_process_noise.png
```

---

## Requisitos

- **ROS 2 Humble** o superior
- **Python 3.8+**
- Dependencias Python (listas en `setup.py`):
  ```bash
  pip install numpy scipy matplotlib
  ```

---

## Instalación y compilación

1. Clonar el repositorio:
   ```bash
   git clone https://github.com/pablodiiaz1/p2_kf_pdc.git
   cd p2_kf_pdc
   ```

2. Compilar el espacio de trabajo ROS 2:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```
---

## Uso

### 1. Iniciar simulación de TurtleBot 4

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=false nav2:=false rviz:=false
```

### 2. Ejecutar nodo de Filtro de Kalman

- **Modelo simplificado (posición)**
  ```bash
  ros2 run p2_kf_pdc kf_estimation --ros-args \
    -p filter.Q:="[0.01, 0, 0; 0, 0.01, 0; 0,0,0.005]" \
    -p filter.R:="[0.05, 0, 0; 0,0.05,0;0,0,0.01]"
  ```

- **Modelo completo (posición + velocidad)**
  ```bash
  ros2 run p2_kf_pdc kf_estimation_vel --ros-args \
    -p filter.Q:="diag([0.02,0.02,0.01,0.01,0.005])" \
    -p filter.R:="diag([0.1,0.1,0.05])"
  ```

> Cada ejemplo muestra parámetros de covarianzas Q y R. Ajusta los valores para experimentar con ruido bajo/alto.

---


## Resultados

Las gráficas generadas se guardan en `graficas/`. Los archivos disponibles incluyen:

- **Modelo simplificado**
  - `simplified_low_noise.png`
  - `simplified_high_measurement_noise.png`
  - `simplified_high_process_noise.png`

- **Modelo completo**
  - `complete_low_noise.png`
  - `complete_high_measurement_noise.png`
  - `complete_high_process_noise.png`

Cada figura muestra la comparación entre la trayectoria real, las mediciones ruidosas y la trayectoria estimada por el filtro.

---


