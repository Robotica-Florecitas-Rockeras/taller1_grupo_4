# 🚀 **TurtleBot2 Control System with ROS2 and CoppeliaSim**

## 📝 **Descripción del Proyecto**  
Este proyecto implementa un sistema de control y monitoreo para el robot diferencial TurtleBot2 en un entorno de simulación utilizando ROS2 y CoppeliaSim. Permite controlar el robot mediante teclado, visualizar su trayectoria en tiempo real y reproducir recorridos previamente guardados. La arquitectura está diseñada con Programación Orientada a Objetos y se optimizó el rendimiento mediante el uso de threads, garantizando la modularidad y escalabilidad del sistema.

---

## 📦 **Estructura del Repositorio**  
```plaintext
turtlebot_project
├── README.md
├── src
│   ├── turtle_bot_teleop
│   │   ├── teleop_node.py
│   ├── turtle_bot_interface
│   │   ├── interface_node.py
│   └── turtle_bot_player
│       ├── player_node.py
├── launch
│   ├── turtle_bot_launch.py
└── docs
    ├── manual_usuario.pdf
    └── informe_tecnico.pdf
```

---

## 🧩 **Características Principales**  
✅ Teleoperación mediante teclado (`W`, `A`, `S`, `D`).  
✅ Visualización gráfica en tiempo real con PyQt5.  
✅ Registro de trayectorias en formato `.json`.  
✅ Reproducción automática de recorridos usando servicios ROS2.  

---

## 💾 **Requisitos del Sistema**  
- Ubuntu 22.04 LTS  
- ROS2 Humble  
- CoppeliaSim  
- Python 3.10 o superior  
- PyQt5  

---

## ⚙️ **Instalación**  

### 1. Clonar el repositorio  
```bash
git clone https://github.com/Robotica-Florecitas-Rockeras/taller1_grupo_4.git
cd turtlebot_project
```

### 2. Configurar el entorno virtual (opcional)  
```bash
python3 -m venv ros2_env
source ros2_env/bin/activate
```

### 3. Instalar las dependencias  
```bash
pip install -r requirements.txt
```

### 4. Compilar el paquete ROS2  
```bash
colcon build
source install/setup.bash
```

---

## 🚀 **Uso del Sistema**  

### **1. Iniciar la simulación en CoppeliaSim**  
1. Abrir CoppeliaSim y cargar la escena `taller1.ttt`.  
2. Verificar que la interfaz para ROS2 esté cargada correctamente.  

### **2. Ejecutar los nodos de ROS2**  
- **Nodo de Teleoperación:**  
```bash
ros2 run turtle_bot_teleop teleop_node
```

- **Nodo de Interfaz Gráfica:**  
```bash
ros2 run turtle_bot_interface interface_node
```

- **Nodo de Reproducción de Trayectorias:**  
```bash
ros2 run turtle_bot_player player_node
```

---

## 🗂️ **Descripción de los Archivos Principales**  
- **`teleop_node.py`**: Captura las teclas presionadas y publica comandos de velocidad en `/turtlebot_cmdVel`.  
- **`interface_node.py`**: Visualiza la trayectoria y permite guardar los datos en un archivo `.json`.  
- **`player_node.py`**: Reproduce los recorridos previamente guardados, enviando comandos al tópico `/turtlebot_cmdVel`.  

---

## 🤝 **Autores**  
- Mateo Chilito Avella - Universidad de los Andes  
- Cristihan Meza Poveda - Universidad de los Andes  
- Samuel Mora Carrizosa - Universidad de los Andes  
- Brayan Joya Herrera - Universidad de los Andes  

---

## 📝 **Licencia**  
Este proyecto está licenciado bajo la Licencia MIT. Consulte el archivo `LICENSE` para más detalles.  

