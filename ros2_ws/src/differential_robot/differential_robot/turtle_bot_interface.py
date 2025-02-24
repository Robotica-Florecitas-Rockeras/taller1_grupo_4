#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget, QLineEdit, QPushButton, QFileDialog, QHBoxLayout, QDialog
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QPixmap, QTransform, QPainter, QPen
from typing import Callable
import json
from functools import partial

from pcl_msgs.srv import UpdateFilename


class TurtleBotInterfaceNode(Node):

    def __init__(self, update_signal, update_orientation_signal, updated_vel_signal, update_simTime_signal):

        super().__init__("turtle_bot_interface")
        self.get_logger().info('Turtle Bot Interface Started\n')
        
        # Señales para actualizar la interfaz
        self.update_signal = update_signal
        self.update_orientation_signal = update_orientation_signal
        self.updated_vel_signal = updated_vel_signal
        self.update_simTime_signal = update_simTime_signal

        self.turtlebot_position = self.create_subscription(Twist, "/turtlebot_position", self.update_turtle_position, 10)
        self.turtlebot_orientation = self.create_subscription(Float32, "/turtlebot_orientation", self.update_turtle_orientation, 10)
        self.turtlebot_vel = self.create_subscription(Twist, "/turtlebot_cmdVel", self.update_trutle_vel, 10)
        self.turtlebot_simTime = self.create_subscription(Float32, "/simulationTime", self.update_turtle_simTime, 10)



    def call_player_service(self, file_path):

        client = self.create_client(UpdateFilename, '/turtlebot_player_service')

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando el servicio...')

        self.get_logger().info('Servicio disponible.')

        req = UpdateFilename.Request()
        req.filename = file_path

        future = client.call_async(req)
        future.add_done_callback(partial(self.done_player_service))


    def done_player_service(self, future):
        try:
            response = future.result()
        except:
            self.get_logger().info("Service Call Failed")


    def update_turtle_position(self, msg: Twist):

        x = msg.linear.x
        y = msg.linear.y
    
        position_data = (x, y)
        
        # Emitimos la señal con la nueva posición
        self.update_signal.emit(position_data)


    def update_turtle_orientation(self, msg:Float32):

        
        #self.get_logger().info(f"Angulo: " + str((msg.data)))

        theta = msg.data # Ángulo en radianes
        self.update_orientation_signal.emit(theta)


    def update_trutle_vel(self, msg: Twist):

        x = msg.linear.x
        y = msg.linear.y
    
        vel_data = (x, y)
        
        # Emitimos la señal con las velocidades
        self.updated_vel_signal.emit(vel_data)

    
    def update_turtle_simTime(self, msg:Float32):

        self.update_simTime_signal.emit(msg.data)




class RosThread(QThread):

    # Señales / Hook de datos para procesamiento ascincrono en la interfaz
    position_updated = pyqtSignal(tuple)  
    orientation_updated = pyqtSignal(float)
    vel_updated = pyqtSignal(tuple) 
    simTime_updated = pyqtSignal(float)

    def run(self):
        rclpy.init()
        self.node = TurtleBotInterfaceNode(self.position_updated, self.orientation_updated, self.vel_updated, self.simTime_updated)  
        rclpy.spin(self.node)
        self.node.destroy_node()
        rclpy.shutdown()



class MainUX(QMainWindow):

    # Constructos de la interfaz

    def __init__(self, parent=None):


        super(MainUX, self).__init__(parent=parent)

        self.simTime_data = 0


        # Ventana principal del recorrido
        self.setFixedSize(500, 500)
        self.setWindowTitle("Turtlebot Position")
        self.move(480, 200)
        self.trayect_file_name = None
        self.file_path = None

        # Lienzo para el recorrido
        self.canvas = QPixmap(self.size())  
        self.canvas.fill(Qt.white)  # Fondo blanco

        # Fondo para pintar el recorrido
        self.background_label = QLabel(self)
        self.background_label.setPixmap(self.canvas)
        self.background_label.setGeometry(0, 0, 500, 500)


        # Ventana / Interfaz
        layout = QVBoxLayout()
        central_widget = QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)


        # Tortuguita :)
        self.turtle_label = QLabel(self)
        self.turtle_pixmap = QPixmap("/home/robotica/Desktop/Taller1/src/turtlebot_controller/turtlebot_controller/turtle.png")  

        if self.turtle_pixmap.isNull():
            print("Error: No se pudo cargar la imagen de la tortuga")

        self.turtle_label.setPixmap(self.turtle_pixmap)
        self.turtle_label.setScaledContents(True)
        self.turtle_label.setFixedSize(50, 50)  # Tamaño de la tortuga

        # Instanciar Tortuguita 
        layout.addWidget(self.turtle_label)


        # Inicializar última posición
        self.last_x = None
        self.last_y = None


        # Boton de guardar ruta
        self.save_line = QPushButton("Guardar", self)
        self.save_line.setFixedSize(80, 35)
        self.save_line.move(5,5)
        self.save_line.setStyleSheet("background-color: #81C784; padding: 2px 8px; border: none; border-radius: 5px")
        self.save_line.clicked.connect(self.save_window)


        # Boton de Cargar trayectoria
        self.load_line = QPushButton("Cargar", self)
        self.load_line.setFixedSize(80, 35)
        self.load_line.move(90,5)
        self.load_line.setStyleSheet("background-color: #F4A261; padding: 2px 8px; border: none; border-radius: 5px")
        self.load_line.clicked.connect(self.load_line_window)


        # Boton de Reset
        self.reset_canvas = QPushButton("Reset", self)
        self.reset_canvas.setFixedSize(80, 35)
        self.reset_canvas.move(415,5)
        self.reset_canvas.setStyleSheet("background-color: #f27a7a; padding: 2px 8px; border: none; border-radius: 5px")
        self.reset_canvas.clicked.connect(self.clear_canvas)


        # Hilo del nodo de ROS
        self.ros_thread = RosThread()
        self.ros_thread.position_updated.connect(self.update_turtle_position)  # Conexión de la posición entre ROS y la interfaz
        self.ros_thread.orientation_updated.connect(self.update_turtle_orientation)
        self.ros_thread.simTime_updated.connect(self.update_simTime)
        self.ros_thread.vel_updated.connect(self.update_data_vel)
        self.ros_thread.start()


        # Ventana inical para guardar trayectoria
        ventana = VentanaEmergente("¿Deseas almacenar la trayectoria del robot?", self.save_game)
        ventana.exec_()



# Guardar trayectoria de TurtleBot en un archivo .json

    def save_game(self, filename):
        
        if not filename:
            filename = "trayecto.json" 
                  
        else:
            filename = filename + ".json"
        
        self.trayect_file_name = filename  # Nombre por defecto si no ingresan uno

        options = QFileDialog.Options()
        file_path, _ = QFileDialog.getSaveFileName(self, "Guardar Archivo", filename, "JSON Files (*.json)", options=options)

        
        if file_path:

            self.file_path = file_path

            data = [{"x": 0, "theta": 0, "simTime": 0}]

            with open(file_path, "w") as file:
                json.dump(data, file, indent=4)
    

    def update_simTime(self, simTime_data):

        self.simTime_data = simTime_data


    def update_data_vel(self, vel_data):
        
        if self.file_path is not None and self.trayect_file_name is not None:

            # Acceder al archivo de self.file_path y reescribirlo con la nueva información
            
            try:
                # Leer el contenido actual del archivo (si existe)
                try:
                    with open(self.file_path, "r") as file:
                        data = json.load(file)  
                except (FileNotFoundError, json.JSONDecodeError):
                    data = []  

                # Agregar nuevos datos
                new_entry = {
                    "x": vel_data[0],
                    "theta": vel_data[1],
                    "sim_time": self.simTime_data
                }

                data.append(new_entry)

      
                with open(self.file_path, "w") as file:
                    json.dump(data, file, indent=4)

            except Exception as e:
                print(f"Error actualizando el archivo: {e}")
        
        else:
            pass
    


# Cargar trayectoria en base a un archivo de trayectoria .json

    def load_line_window(self):

        save_line_wd = VentanaEmergente("Seleccione la trayectoria a cargar (path)", self.load_line_file)
        save_line_wd.exec_()


    def load_line_file(self, filename):

        if not filename:
            filename = "trayecto.json"  

        options = QFileDialog.Options()
        file_path, _ = QFileDialog.getOpenFileName(self, "Abrir Archivo", "", "JSON Files (*.json);;Todos los archivos (*)", options=options)
        
        if file_path:

            self.clear_canvas()

            self.start_client = self.ros_thread.node.call_player_service

            try:
                response = self.start_client(file_path)
            except:
                self.ros_thread.node.get_logger().info("El servicio fallo al leer el path")


# Clear Canvas para Reset y carga de trayectoria en base a un archivo .json

    def clear_canvas(self):
        self.canvas.fill(Qt.white)  
        self.background_label.setPixmap(self.canvas)  
        self.last_x, self.last_y = None, None  


# Guardar gráfigo .png de la trayectoria del TurtleBot

    def save_window(self):

        save_wd = VentanaEmergente("¿Deseas guardar el gráfico del recorrido del robot?", self.save_graphic)
        save_wd.exec_()


    def save_graphic(self, filename):

        if not filename:
            filename = "trayecto.png"  # Nombre por defecto si no ingresan uno
        else:
            filename = filename + ".png"

        options = QFileDialog.Options()
        file_path, _ = QFileDialog.getSaveFileName(self, "Guardar Imagen", filename, "PNG Files (*.png);;JPEG Files (*.jpg)", options=options)
        
        if file_path:
            self.canvas.save(file_path)

    
    def update_turtle_position(self, position_data):

        x, y = position_data

        scale = 100  # 1 metro = 100 píxeles  (Ventana de 500x500 escenario de 5x5)
        window_size = 500  # Tamaño de la ventana en píxeles
        turtle_size = 50  # Tamaño del icono de la tortuga

        # Centrar el (0,0) de la simulación en el centro de la ventana
        screen_x = int((window_size // 2) + x * scale)
        screen_y = int((window_size // 2) - y * scale)  # Invertimos Y

         # Dibujar línea del recorrido
        if self.last_x is not None and self.last_y is not None:
            painter = QPainter(self.canvas)
            pen = QPen(Qt.blue, 2)  # Línea azul de grosor 2px
            painter.setPen(pen)
            painter.drawLine(self.last_x, self.last_y, screen_x, screen_y)
            painter.end()

            # Actualizar la imagen de fondo con el recorrido
            self.background_label.setPixmap(self.canvas)

        # Guardar la posición actual como la última
        self.last_x, self.last_y = screen_x, screen_y

        self.turtle_label.move(screen_x - turtle_size // 2, screen_y - turtle_size // 2)
    

# Actualización de la trayectoria de TurtleBot en la interfaz
    
    def update_turtle_orientation(self, theta):
        self.current_theta = theta  # Guarda el ángulo
        self.update_turtle_rotation()


    def update_turtle_rotation(self):
        transform = QTransform()
        transform.rotate(-(abs(self.current_theta) * 180 / 3.1416) + 90)
        rotated_pixmap = self.turtle_pixmap.transformed(transform, Qt.SmoothTransformation)
        self.turtle_label.setPixmap(rotated_pixmap)



class VentanaEmergente(QDialog):

    def __init__(self, message: str, ok_callback: Callable[[], None] = None,  close_callback: Callable[[], None] = None):
        
        super().__init__()

        self.ok_callback = ok_callback


        # Instancia propiedades de la ventana
        self.setFixedSize(500, 500)
        self.setWindowTitle("Operaciones con archivos")
        self.move(480, 200)

        
        # Layout principalde ventana emergente
        layout = QVBoxLayout()


        # Mensaje de la ventana emergente
        mensaje = QLabel(f"{str(message)}")
        mensaje.setFixedWidth(500)
        mensaje.setAlignment(Qt.AlignCenter)
        layout.addWidget(mensaje)
           

        # Input para el nombre del archivo
        self.filename_input = QLineEdit(self)
        self.filename_input.setPlaceholderText("Nombre del archivo")
        self.filename_input.setFixedHeight(35)
        self.filename_input.setStyleSheet("border-radius: 10px; border: 2px solid gray; padding: 5px")
        self.filename_input.setStyleSheet("padding: 30px; margin: auto")
        layout.addWidget(self.filename_input)  # Campo de texto para el nombre


        # Botón para Guardar / Ejecutar
        self.save_button = QPushButton("Ok", self)
        self.save_button.setFixedSize(150, 35)
        self.save_button.setStyleSheet("background-color: #6fa3f5; padding: 2px 8px; border: none; border-radius: 5px")
        self.save_button.clicked.connect(self.save)


        # Boton para cerrar la ventana emergente
        self.close_button = QPushButton("Cerrar")
        self.close_button.clicked.connect(self.close)
        self.close_button.setFixedSize(150, 35)
        self.close_button.setStyleSheet("background-color: #f27a7a; padding: 2px 8px; border: none; border-radius: 5px")


        # Instanciar el Layout
        user_layout = QHBoxLayout()
        user_layout.setAlignment(Qt.AlignCenter)
        user_layout.addWidget(self.save_button)  # Botón de guardar
        user_layout.addWidget(self.close_button) # Boton Cerrar
        layout.addLayout(user_layout)
        self.setLayout(layout)


    def save(self):

        self.ok_callback(self.filename_input.text().strip())
        
        self.close()



def main():
    try:
        app = QApplication([])
        window = MainUX()
        window.show()
        app.exec_()

    except KeyboardInterrupt:
        print("Pressed Interface Finishing...\n")

    finally:
        print("Interface Finished Successfully :) \n")


if __name__ == '__main__':
    main()
