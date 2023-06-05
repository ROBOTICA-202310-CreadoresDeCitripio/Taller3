import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from pynput import keyboard
import threading
from math import pi, atan2, sqrt
import math
import numpy as np

class RobotManipulatorPlanner(Node):
    def _init_(self):
        super()._init_('robot_manipulator_planner')
        self.publisher_vel = self.create_publisher(Vector3, '/robot_manipulator_goal', 10)
        timer_period = 0.1
        # Definir dimensiones del robot 
        self.dimension1 = 10.5  # dimensión 1 (cm)
        self.dimension2 = 10.5  # dimensión 2 (cm)
        self.dimension3 = 10.5    # dimensión 3 (cm)
        # Se pregunta al usuario que valores desea ingresar
        while True:
            try:
                self.iniciox = 0.0
                self.inicioy = 0.0
                self.inicioz = 0.0
                self.coordinate_x = float(input("Ingrese la coordenada x que desea de la posición del end-effector: "))
                self.coordinate_y = float(input("Ingrese la coordenada y que desea de la posición del end-effector: "))
                self.coordinate_z = float(input("Ingrese la coordenada z que desea de la posición del end-effector: "))
                break
            except ValueError:
                print("Entrada inválida. Por favor ingrese un número.")
        
        self.inverseKinematics()
       
        self.velocity = {'iniciox': 0.0, 'inicioy': 0.0, 'inicioz': 0.0}

    def inverseKinematics(self):
        # Matrices de transformación homogénea
        T = np.array([
            [1, 0, 0, self.coordinate_x],
            [0, 1, 0, self.coordinate_y],
            [0, 0, 1, self.coordinate_z - self.dimension3],
            [0, 0, 0, 1]
        ])

        T0_1 = np.array([
            [np.cos(self.iniciox), -np.sin(self.iniciox), 0, 0],
            [np.sin(self.iniciox), np.cos(self.iniciox), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        T1_2 = np.array([
            [np.cos(self.inicioy), -np.sin(self.inicioy), 0, self.dimension1],
            [0, 0, -1, 0],
            [np.sin(self.inicioy), np.cos(self.inicioy), 0, 0],
            [0, 0, 0, 1]
        ])

        T2_3 = np.array([
            [np.cos(self.inicioz), -np.sin(self.inicioz), 0, self.dimension2],
            [np.sin(self.inicioz), np.cos(self.inicioz), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        T0_3 = np.dot(np.dot(T0_1, T1_2), T2_3)

        # Cálculo de los ángulos de la cinemática inversa
        theta1 = atan2(T0_3[1, 3], T0_3[0, 3])
        theta2 = atan2(sqrt(T0_3[0, 3]*2 + T0_3[1, 3]*2) - self.dimension1, T0_3[2, 3])
        theta3 = atan2(T0_3[2, 2], -T0_3[2, 0])

        # Convertir ángulos a valores entre -pi y pi
        theta1 = theta1 % (2 * pi)
        if theta1 > pi:
            theta1 -= 2 * pi

        theta2 = theta2 % (2 * pi)
        if theta2 > pi:
            theta2 -= 2 * pi

        theta3 = theta3 % (2 * pi)
        if theta3 > pi:
            theta3 -= 2 * pi

        gRot = math.degrees(theta1)
        gj1 = math.degrees(theta2)
        gj2 = math.degrees(theta3)

        message = Vector3()
        message.x = gRot
        message.y = gj1
        message.z = gj2
        self.publisher_vel.publish(message)

def main():
    rclpy.init()
    node = RobotManipulatorPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if _name_ == '_main_':
    main()