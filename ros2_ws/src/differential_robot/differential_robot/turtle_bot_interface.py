#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget, QLineEdit, QPushButton, QFileDialog, QHBoxLayout
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QPixmap, QTransform, QPainter, QPen


class TurtleBotInterfaceNode(Node):
    def __init__(self, update_signal, update_orientation_signal):
        super().__init__("turtle_bot_interface")
        self.get_logger().info('Turtle Bot Interface Started\n')
        
        # Señales para actualizar la interfaz
        self.update_signal = update_signal
        self.update_orientation_signal = update_orientation_signal

        self.turtlebot_position = self.create_subscription(Twist, "/turtlebot_position", self.update_turtle_position, 10)
        self.turtlebot_orientation = self.create_subscription(Float32, "/turtlebot_orientation", self.update_turtle_orientation, 1)

    def update_turtle_position(self, msg: Twist):

        x = msg.linear.x
        y = msg.linear.y
    
        position_data = (x, y)
        
        # Emitimos la señal con la nueva posición
        self.update_signal.emit(position_data)

    def update_turtle_orientation(self, msg:Float32):

        
        self.get_logger().info(f"Angulo: " + str((msg.data)))

        theta = msg.data # Ángulo en radianes

        self.update_orientation_signal.emit(theta)

    

class RosThread(QThread):
    position_updated = pyqtSignal(tuple)  # Señal / Hook de datos
    orientation_updated = pyqtSignal(float)

    def run(self):
        rclpy.init()
        self.node = TurtleBotInterfaceNode(self.position_updated, self.orientation_updated)  
        rclpy.spin(self.node)
        self.node.destroy_node()
        rclpy.shutdown()



class MainUX(QMainWindow):
    def __init__(self, parent=None):
        super(MainUX, self).__init__(parent=parent)

        self.setFixedSize(500, 500)
        self.setWindowTitle("Turtlebot Position")

        # Lienzo para el recorrido
        self.canvas = QPixmap(self.size())  
        self.canvas.fill(Qt.white)  # Fondo blanco

        self.background_label = QLabel(self)
        self.background_label.setPixmap(self.canvas)
        self.background_label.setGeometry(0, 0, 500, 500)


        # Tortuguita :)
        self.turtle_label = QLabel(self)
        self.turtle_pixmap = QPixmap("/home/robotica/Desktop/taller1_grupo_4/ros2_ws/src/differential_robot/differential_robot/turtle.png")  
        if self.turtle_pixmap.isNull():
            print("Error: No se pudo cargar la imagen de la tortuga")
        self.turtle_label.setPixmap(self.turtle_pixmap)
        self.turtle_label.setScaledContents(True)
        self.turtle_label.setFixedSize(50, 50)  # Tamaño de la tortuga

         # Input para el nombre del archivo
        self.filename_input = QLineEdit(self)
        self.filename_input.setPlaceholderText("Nombre de la gráfica")
        self.filename_input.setFixedSize(100, 25)

        # Botón para guardar la imagen
        self.save_button = QPushButton("Guardar Gráfica", self)
        self.save_button.setFixedSize(100, 25)
        self.save_button.clicked.connect(self.save_graphic)



        
        # Inicializar última posición
        self.last_x = None
        self.last_y = None
        
        # Ventana / Interfaz
        layout = QVBoxLayout()
        central_widget = QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)


        # Instanciar Tortuguita 
        layout.addWidget(self.turtle_label)


        # Instancia elemento del usuario
        layout.addWidget(self.filename_input)  # Campo de texto para el nombre
        layout.addWidget(self.save_button)  # Botón de guardar


        # Hilo del nodo de ROS
        self.ros_thread = RosThread()
        self.ros_thread.position_updated.connect(self.update_turtle_position)  # Conexión de la posición entre ROS y la interfaz
        self.ros_thread.orientation_updated.connect(self.update_turtle_orientation)
        self.ros_thread.start()


    def save_graphic(self):
        filename = self.filename_input.text().strip()
        if not filename:
            filename = "trayecto"  # Nombre por defecto si no ingresan uno

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
    

    def update_turtle_orientation(self, theta):
        self.current_theta = theta  # Guarda el ángulo
        self.update_turtle_rotation()


    def update_turtle_rotation(self):
        transform = QTransform()
        transform.rotate(self.current_theta * 180 / 3.1416)
        rotated_pixmap = self.turtle_pixmap.transformed(transform, Qt.SmoothTransformation)
        self.turtle_label.setPixmap(rotated_pixmap)


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
