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
        timer_period= 0.1
        # Definir dimensiones del robot 
        self.l1 = 102.4 # dimensión 1 (cm)
        self.l2 = 138.6  # dimensión 2 (cm)
        self.l3 = 155  # dimensión 3 (cm)
        # Se pregunta al usuario que valores desea ingresar
        while True:
            try:
                self.hola = 0.0
                self.como = 0.0
                self.estas = 0.0
                self.x = float(input("Ingrese la coordenada x que desea de la posición del end-effector: "))
                self.y = float(input("Ingrese la coordenada y que desea de la posición del end-effector: "))
                self.z = float(input("Ingrese la coordenada z que desea de la posición del end-effector: "))
                break
            except ValueError:
                print("Entrada inválida. Por favor ingrese un número.")
        
        self.cinematicaInversa()
       
        self.velocity = {'hola' : 0.0, 'como' : 0.0, 'estas' : 0.0}

    def cinematicaInversa(self):
        # Matrices de transformación homogénea
        T = np.array([
            [1, 0, 0, self.x],
            [0, 1, 0, self.y],
            [0, 0, 1, self.z - self.l3],
            [0, 0, 0, 1]
        ])

        T0_1 = np.array([
            [np.cos(self.hola), -np.sin(self.hola), 0, 0],
            [np.sin(self.hola), np.cos(self.hola), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        T1_2 = np.array([
            [np.cos(self.como), -np.sin(self.como), 0, self.l1],
            [0, 0, -1, 0],
            [np.sin(self.como), np.cos(self.como), 0, 0],
            [0, 0, 0, 1]
        ])

        T2_3 = np.array([
            [np.cos(self.estas), -np.sin(self.estas), 0, self.l2],
            [np.sin(self.estas), np.cos(self.estas), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        T0_3 = np.dot(np.dot(T0_1, T1_2), T2_3)

        # Cálculo de los ángulos de la cinemática inversa
        theta1 = atan2(T0_3[1, 3], T0_3[0, 3])
        theta2 = atan2(sqrt(T0_3[0, 3]*2 + T0_3[1, 3]*2) - self.l1, T0_3[2, 3])
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
    node.destroy_node
    rclpy.shutdown()

if __name__ == '_main_':
    main()