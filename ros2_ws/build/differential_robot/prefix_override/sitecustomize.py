import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/robotica/Downloads/Taller1/taller1_grupo_4/ros2_ws/install/differential_robot'
