# 🚀 **TurtleBot2 Control System with ROS2 and CoppeliaSim**

## 📝 **Descripción del Proyecto**  
Este proyecto implementa un sistema de control y monitoreo para el robot diferencial TurtleBot2 en un entorno de simulación utilizando ROS2 y CoppeliaSim. Permite controlar el robot mediante teclado, visualizar su trayectoria en tiempo real y reproducir recorridos previamente guardados. La arquitectura está diseñada con principios de Programación Orientada a Objetos y se optimizó el rendimiento mediante threads, garantizando la modularidad y escalabilidad del sistema.

---

## 📦 **Estructura del Repositorio**  
```plaintext
ros2_ws/src/differential_robot/differential_robot
├── README.md
├── __init__.py
├── turtle.png
├── turtle_bot_interface.py
├── turtle_bot_player.py
└── turtle_bot_teleop.py
```

---

## 💾 **Requisitos del Sistema**  
- Ubuntu 22.04 LTS  
- ROS2 Humble  
- CoppeliaSim  
- Python 3.10 o superior  
- PyQt5  

---

## 🚀 **Uso del Sistema**  

### **1. Iniciar la simulación en CoppeliaSim**  
1. Abrir CoppeliaSim y cargar la escena `taller1.ttt`.  
2. Verificar que la interfaz para ROS2 esté cargada correctamente.  

### **2. Ejecutar los nodos de ROS2**  
- **Nodo de Teleoperación:**  
```bash
ros2 run differential_robot turtle_bot_teleop
```

- **Nodo de Interfaz Gráfica:**  
```bash
ros2 run differential_robot turtle_bot_interface
```

- **Nodo de Reproducción de Trayectorias:**  
```bash
ros2 run differential_robot turtle_bot_player
```

## 🤝 **Autores**  
- Mateo Chilito Avella - Universidad de los Andes  
- Cristihan Meza Poveda - Universidad de los Andes  
- Samuel Mora Carrizosa - Universidad de los Andes  
- Brayan Joya Herrera - Universidad de los Andes  

---

## 📝 **Licencia**  
Este proyecto está licenciado bajo la Licencia MIT. Consulte el archivo `LICENSE` para más detalles.  

---
